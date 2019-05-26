#include "wumpus/wm/WumpusSimData.h"

#include "wumpus/WumpusWorldModel.h"
#include "wumpus/model/Agent.h"
#include "wumpus/model/Field.h"
#include "wumpus/wm/PlanningModule.h"
#include <WumpusEnums.h>

#include <SystemConfig.h>
#include <engine/AlicaClock.h>

#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <utility>

namespace wumpus
{
namespace wm
{

WumpusSimData::WumpusSimData(WumpusWorldModel* wm)
{
    this->wm = wm;
    auto sc = this->wm->getSystemConfig();

    // data buffers
    this->actionResponseValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.ValidityDuration", NULL));
    this->actionResponseBuffer =
            new supplementary::InfoBuffer<wumpus_simulator::ActionResponse>((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.BufferLength", NULL));
    this->turnInfoValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.TurnInfo.ValidityDuration", NULL));
    this->isMyTurnBuffer = new supplementary::InfoBuffer<bool>((*sc)["WumpusWorldModel"]->get<int>("Data.TurnInfo.BufferLength", NULL));
    this->integratedFromSimulator = false;

}

WumpusSimData::~WumpusSimData() = default;

/**
* Adds new agent to list of known agents
* Sets Playground size if necessary
*/
void WumpusSimData::processInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse)
{

    //avoid inconsistencies in knowledgebase and crashing of clingo
    if(this->wm->localAgentDied) {
        return;
    }

    auto id = initialPoseResponse->agentId;

    this->wm->playground->initializePlayground(initialPoseResponse->fieldSize);
    auto agent = this->wm->playground->getAgentById(id);
    if (!agent) {
        agent = std::make_shared<wumpus::model::Agent>(this->wm->changeHandler, initialPoseResponse->agentId);
        this->wm->playground->addAgent(agent);
        agent->registerAgent(id == essentials::SystemConfig::getInstance()->getOwnRobotID());
    }

    // any other info should come from the agent itself
    if (initialPoseResponse->agentId != this->wm->getSystemConfig()->getOwnRobotID()) {
        return;
    }

    auto field = this->wm->playground->getField(initialPoseResponse->x, initialPoseResponse->y);
    field->updateVisited(true);

    agent->updatePosition(field);
    agent->updateInitialPosition(field);
    agent->updateHeading(initialPoseResponse->heading);
    agent->updateArrow(initialPoseResponse->hasArrow);
}

void WumpusSimData::setIntegratedFromSimulator(bool integrated)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->integratedFromSimulator = integrated;
}

bool WumpusSimData::getIntegratedFromSimulator()
{
    std::lock_guard<std::mutex> lock(mtx);
    return this->integratedFromSimulator;
}

void WumpusSimData::processActionResponse(wumpus_simulator::ActionResponsePtr actionResponse)
{

    // reject msgs which aren't meant for me
    if (actionResponse->agentId != this->wm->getSystemConfig()->getOwnRobotID()) {
        return;
    }

    // Add info Element to Buffer
    auto actionResponseInfo = std::make_shared<supplementary::InformationElement<wumpus_simulator::ActionResponse>>(
            *(actionResponse.get()), wm->getTime(), this->actionResponseValidityDuration, 1.0);
    actionResponseBuffer->add(actionResponseInfo);

    //avoid inconsistencies in knowledgebase and crashing of clingo
    if(responsesContain(actionResponse->responses,WumpusEnums::dead)) {
        this->wm->localAgentDied = true;
        return;
    }

    auto field = this->wm->playground->getField(actionResponse->x, actionResponse->y);
    auto agent = this->wm->playground->getAgentById(actionResponse->agentId);

    if (!agent) {
        agent = std::make_shared<wumpus::model::Agent>(this->wm->changeHandler, actionResponse->agentId);
        this->wm->playground->addAgent(agent);
        agent->updateArrow(true);
    }
    agent->updatePosition(field);
    agent->updateHeading(actionResponse->heading);
    agent->updateHaveGold(responsesContain(actionResponse->responses, WumpusEnums::goldFound));

    field->updateDrafty(responsesContain(actionResponse->responses, WumpusEnums::drafty));
    field->updateShiny(responsesContain(actionResponse->responses, WumpusEnums::shiny));
    field->updateStinky(responsesContain(actionResponse->responses, WumpusEnums::stinky));
    field->updateVisited(true);

    if (responsesContain(actionResponse->responses, WumpusEnums::silence)) {
        this->wm->playground->handleSilence();
    }

    if (responsesContain(actionResponse->responses, WumpusEnums::scream)) {
        this->wm->playground->handleScream();
    }

    auto turnInfo = std::make_shared<supplementary::InformationElement<bool>>(
            actionResponse->agentId == essentials::SystemConfig::getOwnRobotID() && responsesContain(actionResponse->responses, WumpusEnums::yourTurn),
            wm->getTime(), this->turnInfoValidityDuration, 1.0);

    this->isMyTurnBuffer->add(turnInfo);

    //yourTurn message is the last message the Simulator sends
    if (actionResponse->agentId == essentials::SystemConfig::getOwnRobotID() && responsesContain(actionResponse->responses, WumpusEnums::yourTurn)) {
        this->integratedFromSimulator = true;

    }

    if(responsesContain(actionResponse->responses, WumpusEnums::exited)) {
        this->wm->localAgentExited = true;
    }


}

void WumpusSimData::processAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception)
{
    auto agent = this->wm->playground->getAgentById(agentPerception->senderID);
    if (!agent) {
        agent = std::make_shared<wumpus::model::Agent>(this->wm->changeHandler, agentPerception->senderID);
        this->wm->playground->addAgent(agent);
    }

    auto field = this->wm->playground->getField(agentPerception->position.x, agentPerception->position.y);
    field->updateDrafty(agentPerception->drafty);
    field->updateShiny(agentPerception->glitter);
    field->updateStinky(agentPerception->stinky);
    field->updateVisited(true);
    if(this->integratedFromAgents.find(agentPerception->senderID) == this->integratedFromAgents.end()) {
        this->integratedFromAgents.emplace(agentPerception->senderID,true);
    } else {
        this->integratedFromAgents.at(agentPerception->senderID) = true;
    }
}

/**
 * Clear buffers to be able to start a new Base from a running program (evaluation scenario)
 */
void WumpusSimData::clearBuffers() {
    this->actionResponseBuffer->clear(true);
    this->isMyTurnBuffer->clear(true);
}

const supplementary::InfoBuffer<wumpus_simulator::ActionResponse>* WumpusSimData::getActionResponseBuffer()
{
    return this->actionResponseBuffer;
}
const supplementary::InfoBuffer<bool>* WumpusSimData::getIsMyTurnBuffer()
{
    return this->isMyTurnBuffer;
}

void WumpusSimData::resetIntegratedFromAgents() {
    for(auto& entry : this->integratedFromAgents) {
        entry.second = false;
    }
}
}
} /* namespace wumpus */
