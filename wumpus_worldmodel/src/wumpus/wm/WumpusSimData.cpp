#include "wumpus/wm/WumpusSimData.h"

#include "wumpus/WumpusWorldModel.h"
#include "wumpus/model/Agent.h"
#include "wumpus/model/Field.h"
#include "wumpus/wm/PlanningModule.h"
#include <WumpusEnums.h>

#include <SystemConfig.h>
#include <engine/AgentIDConstPtr.h>
#include <engine/AlicaClock.h>
#include <engine/IRoleAssignment.h>
#include <engine/teammanager/TeamManager.h>
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

    this->turn = 1;

    // data buffers
    this->actionResponseValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.ValidityDuration", NULL));
    this->actionResponseBuffer =
            new supplementary::InfoBuffer<wumpus_simulator::ActionResponse>((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.BufferLength", NULL));
    this->turnInfoValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.TurnInfo.ValidityDuration", NULL));
    this->isMyTurnBuffer = new supplementary::InfoBuffer<bool>((*sc)["WumpusWorldModel"]->get<int>("Data.TurnInfo.BufferLength", NULL));

    this->integratedFromSimulatorForTurnNumber = false;
}

WumpusSimData::~WumpusSimData() = default;

/**
* Adds new agent to list of known agents
* Sets Playground size if necessary
*/
void WumpusSimData::processInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse)
{
    //    std::cout << "WumpusSimData: in initial pose response " << std::endl;
    // avoid inconsistencies in knowledgebase and crashing of clingo

    std::lock_guard<std::mutex> lock(this->respMtx);
    if (this->wm->localAgentDied) {
        return;
    }

    auto id = initialPoseResponse->agentId;

    this->wm->playground->initializePlayground(initialPoseResponse->fieldSize);
    auto agent = this->wm->playground->getAgentById(id);
    if (!agent) {
        agent = std::make_shared<wumpus::model::Agent>(this->wm->changeHandler, initialPoseResponse->agentId);
        this->wm->playground->addAgent(agent);
        this->wm->playground->addAgentForExperiment(agent);
        agent->registerAgent(id == essentials::SystemConfig::getOwnRobotID());
    }

    // any other info should come from the agent itself
    if (initialPoseResponse->agentId != this->wm->getSystemConfig()->getOwnRobotID()) {
        return;
    }

    auto field = this->wm->playground->getField(initialPoseResponse->x, initialPoseResponse->y);
    field->updateVisited(true);
    field->updateExplored(true);

    agent->updatePosition(field);
    agent->updateInitialPosition(field);
    agent->updateHeading(initialPoseResponse->heading);
    agent->updateArrow(initialPoseResponse->hasArrow);
}

void WumpusSimData::setIntegratedFromSimulator(bool integrated)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->integratedFromSimulatorForTurnNumber = integrated;
}

bool WumpusSimData::getIntegratedFromSimulator()
{
    std::lock_guard<std::mutex> lock(mtx);
    return this->integratedFromSimulatorForTurnNumber;
}

void WumpusSimData::processActionResponse(wumpus_simulator::ActionResponsePtr actionResponse)
{
    // FIXME process one message at a time for now - clingo keeps crashing
//    std::lock_guard<std::mutex> lock(this->respMtx);
    // ignore msgs which aren't meant for me
    if (actionResponse->agentId != this->wm->getSystemConfig()->getOwnRobotID()) {
        if (this->turn > 0) {
            auto id = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
            // only one agent responsible for writing logs (to make testing on local machine easier)
            // each turn multiple actionresponses are sent so only count for the "yourTurn" messages

            if (this->wm->getEngine()->getRoleAssignment()->getRole(id)->getName() == this->wm->spawnRequestHandlerRoleName)
                if(responsesContain(actionResponse->responses, WumpusEnums::yourTurn)) {
                    this->wm->experiment->getCurrentRun()->getCurrentResult()->increaseActionsCostCounter(
                            actionResponse->agentId); // TODO only tracks other agents' actions - own actions are currently tracked in
                    // performaction beh0
                if(responsesContain(actionResponse->responses,WumpusEnums::exited)) {

                }
            }
        }
        return;
    }

    // Add info Element to Buffer
    auto actionResponseInfo = std::make_shared<supplementary::InformationElement<wumpus_simulator::ActionResponse>>(
            *(actionResponse.get()), wm->getTime(), this->actionResponseValidityDuration, 1.0);
    actionResponseBuffer->add(actionResponseInfo);

    // avoid inconsistencies in knowledgebase and crashing of clingo
    if (responsesContain(actionResponse->responses, WumpusEnums::dead)) {
        this->wm->localAgentDied = true;
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->setDead();
        return;
    }

    if (responsesContain(actionResponse->responses, WumpusEnums::exited)) {
        this->wm->localAgentExited = true;
        //        std::cout << "WumpusSumData: local agent exited " << std::endl;
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->setExited();
        return; // needed for exitting after shooting
    }
    std::cout << "FIELD: " << actionResponse->x << ", " << actionResponse->y << std::endl;
    auto field = this->wm->playground->getField(actionResponse->x, actionResponse->y);
    auto agent = this->wm->playground->getAgentById(actionResponse->agentId);

    if (!agent) {
        agent = std::make_shared<wumpus::model::Agent>(this->wm->changeHandler, actionResponse->agentId);
        this->wm->playground->addAgent(agent);
        this->wm->playground->addAgentForExperiment(agent);
        agent->registerAgent(actionResponse->agentId == essentials::SystemConfig::getOwnRobotID());
        agent->updateArrow(true);
    }
    agent->updatePosition(field);
    agent->updateHeading(actionResponse->heading);
    agent->updateHaveGold(responsesContain(actionResponse->responses, WumpusEnums::goldFound));

    field->updateDrafty(responsesContain(actionResponse->responses, WumpusEnums::drafty));
    field->updateShiny(responsesContain(actionResponse->responses, WumpusEnums::shiny));
    field->updateStinky(responsesContain(actionResponse->responses, WumpusEnums::stinky));
    field->updateExplored(true);
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

    //    std::cout << "TURNINFO: "<<turnInfo->getInformation();

    this->isMyTurnBuffer->add(turnInfo);

    // yourTurn message is the last message the Simulator sends
    if (actionResponse->agentId == essentials::SystemConfig::getOwnRobotID() && responsesContain(actionResponse->responses, WumpusEnums::yourTurn)) {
        this->integratedFromSimulatorForTurnNumber = true; // this->turn;
        this->raiseTurnCounter();
    }


}

void WumpusSimData::processAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception)
{
    std::lock_guard<std::mutex> lock(this->wm->resetMtx); //FIXME
    std::lock_guard<std::mutex> lock2(this->respMtx); // FIXME fix segfault rc properly

    if(!agentPerception->encoding.empty() && agentPerception->encoding != this->wm->currentEncoding) {
        this->wm->currentEncoding = agentPerception->encoding;
        if(this->wm->resettedForEncoding.find(agentPerception->encoding) != this->wm->resettedForEncoding.end()) {
            return;
        }
    }

    // allow communicating of dead/exited for result logging
    auto id = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
    if (this->wm->getEngine()->getRoleAssignment()->getRole(id)->getName() == this->wm->spawnRequestHandlerRoleName) {

        if (agentPerception->exited) {
            // at least one agent needs to exit for success
            this->wm->experiment->getCurrentRun()->getCurrentResult()->exited.insert(agentPerception->senderID);
            if (this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus != eval::CompletionStatus::TIMEOUT) {
                this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus = eval::CompletionStatus::SUCCESS;
            }
        }

        if (agentPerception->died) {
            this->wm->experiment->getCurrentRun()->getCurrentResult()->died.insert(agentPerception->senderID);
            if (this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus == eval::CompletionStatus::UNDEFINED) {
                this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus = eval::CompletionStatus::FAILURE;
            }
        }
    }

    if(this->wm->localAgentExited || this->wm->localAgentDied) {
        return;
    }

    if (!this->wm->experiment->communicationAllowed) {
        return;
    }

    // nothing new should come from own perception
    if (agentPerception->senderID == essentials::SystemConfig::getOwnRobotID()) {
        return;
    }

    auto agent = this->wm->playground->getAgentById(agentPerception->senderID);
    if (!agent) {
        return;
        // TODO this might not be a good idea in case one agent already exited
        //        if (!(agentPerception->died || agentPerception->exited)) {
        //            agent = std::make_shared<wumpus::model::Agent>(this->wm->changeHandler, agentPerception->senderID);
        //            this->wm->playground->addAgent(agent);
        //            this->wm->playground->addAgentForExperiment(agent);
        //        }
    }
    //    std::cout << "Processing agent perception" << std::endl;

    if (this->integratedFromOtherAgentsForTurnNr.find(agentPerception->senderID) == this->integratedFromOtherAgentsForTurnNr.end()) {
        //        std::cout << "IFA: couldn't find" << agentPerception->senderID << std::endl;
        if (!(agentPerception->exited || agentPerception->died)) {
            //            std::cout << "IFA: adding " << agentPerception->senderID << std::endl;
            this->integratedFromOtherAgentsForTurnNr.emplace(agentPerception->senderID, true);
        }
    } else {
        this->integratedFromOtherAgentsForTurnNr.at(agentPerception->senderID) = true;
    }

    //    std::cout << "integrated from agents: " << std::endl;
    //    for (auto& elem : this->integratedFromOtherAgentsForTurnNr) {
    //        std::cout << elem.first << ", exited: " << agentPerception->exited << ", died: " << agentPerception->died << std::endl;
    //    }

    // remove agent if they exited or died - after this, no field info should be updated if the agent exited
    if (this->integratedFromOtherAgentsForTurnNr.find(agentPerception->senderID) != this->integratedFromOtherAgentsForTurnNr.end() &&
            (agentPerception->exited || agentPerception->died)) {
        auto agent = this->wm->playground->getAgentById(agentPerception->senderID);
        if (agentPerception->died) {
            agent->setDead();
            //            std::cout << "WSD: Agent Dead!" << std::endl;
            this->integratedFromOtherAgentsForTurnNr.erase(agentPerception->senderID);
        } else if (agentPerception->exited) {
            agent->setExited();
            //            std::cout << "Agent " << agentPerception->senderID << " exited!" << std::endl;
            this->integratedFromOtherAgentsForTurnNr.erase(agentPerception->senderID);
            return; // TODO special case
        }
    }

    std::cout << "############ WUMPUSSIMDATA: UPDATE STUFF FROM AGENT PERCEPTION " << std::endl;

    if (agentPerception->position.x != -1 && agentPerception->position.y != -1) {
        auto playground = this->wm->playground;
        if (!playground) {
            std::cout << "No playground!" << std::endl;
            return;
        }
        auto field = playground->getField(agentPerception->position.x, agentPerception->position.y);
        if (field) {

            field->updateDrafty(agentPerception->drafty);
            field->updateShiny(agentPerception->glitter);
            field->updateStinky(agentPerception->stinky);
            field->updateExplored(true);
        } else {
            std::cout << "WumpusSimData: No field with given coordinates! " << std::endl;
        }
    }

    std::cout << "################WUMPUSSIMDATA: DONE UPDATING" << std::endl;


}


/**
 * Clear buffers to be able to start a new Base from a running program (evaluation scenario)
 */
void WumpusSimData::clearBuffers()
{
    this->actionResponseBuffer->clear(true);
    this->isMyTurnBuffer->clear(true);
    this->integratedFromOtherAgentsForTurnNr.clear();
    this->turn = 1;
}

const supplementary::InfoBuffer<wumpus_simulator::ActionResponse>* WumpusSimData::getActionResponseBuffer()
{
    return this->actionResponseBuffer;
}
const supplementary::InfoBuffer<bool>* WumpusSimData::getIsMyTurnBuffer()
{
    return this->isMyTurnBuffer;
}

void WumpusSimData::resetIntegratedFromAgents()
{
    for (auto& entry : this->integratedFromOtherAgentsForTurnNr) {
        entry.second = false;
    }
}

bool WumpusSimData::isIntegratedFromAllAgents()
{
    for (auto integrated : this->integratedFromOtherAgentsForTurnNr) {
        if (!integrated.second) {
            std::cout << "sensory information from " << integrated.first << "is not integrated yet!" << std::endl;
            return false;
        }
    }
    return true;
}

void WumpusSimData::raiseTurnCounter()
{
    ++this->turn;
}

}
} /* namespace wumpus */
