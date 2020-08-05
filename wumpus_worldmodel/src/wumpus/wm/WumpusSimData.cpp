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
#include <eval/AgentInfo.h>
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
        : integratedFromSimulatorForTurnNumber(false)
        , turn(1)
        , awaitingScreamOrSilence(0)
        , timesCompletionStatusApplied(0)
        , isAwaitingShootingFeedback(false)

{
    this->wm = wm;
    auto sc = this->wm->getSystemConfig();
    this->communicationAllowed = (*sc)["WumpusWorldModel"]->get<bool>("Agents.allowCommunication", NULL);

    // data buffers
    this->actionResponseValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.ValidityDuration", NULL));
    this->actionResponseBuffer =
            new supplementary::InfoBuffer<wumpus_simulator::ActionResponse>((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.BufferLength", NULL));
    this->turnInfoValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.TurnInfo.ValidityDuration", NULL));
    this->isMyTurnBuffer = new supplementary::InfoBuffer<bool>((*sc)["WumpusWorldModel"]->get<int>("Data.TurnInfo.BufferLength", NULL));
}

WumpusSimData::~WumpusSimData()
{
    delete this->actionResponseBuffer;
    delete this->isMyTurnBuffer;
}

/**
* Adds new agent to list of known agents
* Sets Playground size if necessary
*/
void WumpusSimData::processInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse)
{
    //    std::cout << "WumpusSimData: in initial pose response " << std::endl;
    // avoid inconsistencies in knowledgebase and crashing of clingo

    std::lock_guard<std::mutex> lock(this->respMtx);
    auto localId = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
    if (this->wm->getEngine()->getRoleAssignment()->getRole(localId)->getName() == this->wm->spawnRequestHandlerRoleName) {
        auto result = this->wm->experiment->getCurrentRun()->getCurrentResult();
        result->updateAgentInfo(initialPoseResponse->agentId, std::make_pair(initialPoseResponse->x, initialPoseResponse->y), false, false);
    }
    //    this->completionStatusApplied = 0;

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
        if (!this->communicationAllowed) {
            auto pos = this->wm->playground->getField(initialPoseResponse->x, initialPoseResponse->y);
            //            agent->updateInitialPosition(pos, false);
            //            std::cout << "added agent for experiment with id " << initialPoseResponse->agentId << "and initialPose" << initialPoseResponse->x <<
            //            ", "
            //                      << initialPoseResponse->y << std::endl;
            //            if (wumpus::model::Playground::agentsForExperiment.size() == this->wm->getPresetAgentCount()) {
            //                throw std::exception();
            //            }
        }
        if (id == essentials::SystemConfig::getOwnRobotID() || this->communicationAllowed) {
            agent->registerAgent(id == essentials::SystemConfig::getOwnRobotID());
        }
    }

    // any other info should come from the agent itself
    if (initialPoseResponse->agentId != this->wm->getSystemConfig()->getOwnRobotID()) {
        return;
    }

    auto field = this->wm->playground->getField(initialPoseResponse->x, initialPoseResponse->y);
    field->updateVisited(true, essentials::SystemConfig::getOwnRobotID());
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
    // FIXME possible clash actionresponse <-> agentperception?
    std::lock_guard<std::mutex> lock(this->respMtx);
    //    std::lock_guard<std::mutex> lock2(this->wm->resetMtx);

    // FIXME does this have any effect?
    //    if (!this->wm->currentEncoding.empty() && this->wm->resettedForEncoding.find(this->wm->currentEncoding) != this->wm->resettedForEncoding.end()) {
    //        std::cout << "Already resetted for this encoding!" << this->wm->currentEncoding << std::endl;
    //        return;
    //    }

    auto localId = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
    if (this->wm->getEngine()->getRoleAssignment()->getRole(localId)->getName() == this->wm->spawnRequestHandlerRoleName) {
        auto result = this->wm->experiment->getCurrentRun()->getCurrentResult();
        result->updateAgentInfo(actionResponse->agentId, responsesContain(actionResponse->responses, WumpusEnums::dead),
                responsesContain(actionResponse->responses, WumpusEnums::exited));
    }

    // TODO access
    auto me = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID()); // may have been resetted alredy
    if (me) {
        // silence and scream can be heard from anyone
        if (actionResponse->agentId == me->id && responsesContain(actionResponse->responses, WumpusEnums::silence)) {
            me->replanNecessary = true;
            this->wm->playground->handleSilence();
            std::cout << "WSD: got silence" << std::endl;
            me->receivedSilenceForOwnShot = true;
            this->setIsAwaitingShootingFeedback(false);
            //            this->decrementAwaitingScreamOrSilence();
        }

        if (actionResponse->agentId == me->id && responsesContain(actionResponse->responses, WumpusEnums::scream)) {
            me->replanNecessary = true;
            this->wm->playground->handleScream();
            std::cout << "WSD: got scream" << std::endl;
            me->receivedScreamForOwnShot = true;

            this->setIsAwaitingShootingFeedback(false);
            //            this->decrementAwaitingScreamOrSilence();
        }
    }

    auto id = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
    // only one agent responsible for writing logs (to make testing on local machine easier)
    // each turn multiple actionresponses are sent so only count for the "yourTurn" messages
    std::cout << "set completion status" << std::endl;
    if (this->wm->getEngine()->getRoleAssignment()->getRole(id)->getName() == this->wm->spawnRequestHandlerRoleName) {
        if (responsesContain(actionResponse->responses, WumpusEnums::exited)) {
            this->wm->experiment->getCurrentRun()->getCurrentResult()->exited.insert(actionResponse->agentId);
            this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus = eval::CompletionStatus::SUCCESS;
            //++this->timesCompletionStatusApplied;
        } else if (responsesContain(actionResponse->responses, WumpusEnums::dead)) {
            this->wm->experiment->getCurrentRun()->getCurrentResult()->died.insert(actionResponse->agentId);
            if (this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus != eval::CompletionStatus::SUCCESS) {
                this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus = eval::CompletionStatus::FAILURE;
            }
            //++this->timesCompletionStatusApplied;
        }
    }

    // ignore msgs which aren't meant for me
    if (actionResponse->agentId != this->wm->getSystemConfig()->getOwnRobotID()) {
        // only one agent responsible for writing logs (to make testing on local machine easier)
        // each turn multiple actionresponses are sent so only count for the "yourTurn" messages

        if (this->wm->getEngine()->getRoleAssignment()->getRole(id)->getName() == this->wm->spawnRequestHandlerRoleName) {
            if (responsesContain(actionResponse->responses, WumpusEnums::yourTurn)) {
                std::cout << "raise actions cost counter " << actionResponse->agentId << std::endl;
                this->wm->experiment->getCurrentRun()->getCurrentResult()->increaseActionsCostCounter(
                        actionResponse->agentId); // TODO only tracks other agents' actions - own actions are currently tracked in
                // performaction beh0
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
        this->wm->extractor->writeGetSolutionStatsReusable("", -1, -1);
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())
                ->setDead(this->wm->playground->getField(actionResponse->x, actionResponse->y));
        this->wm->localAgentDied = true;

        return;
    }

    if (responsesContain(actionResponse->responses, WumpusEnums::exited)) {
        this->wm->localAgentExited = true;
        std::cout << "WumpusSumData: local agent exited " << std::endl;
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->setExited();

        return; // needed for exitting after shooting
    }
    auto field = this->wm->playground->getField(actionResponse->x, actionResponse->y);
    auto agent = this->wm->playground->getAgentById(actionResponse->agentId);

    if (!agent) {
        agent = std::make_shared<wumpus::model::Agent>(this->wm->changeHandler, actionResponse->agentId);
        this->wm->playground->addAgent(agent);
        this->wm->playground->addAgentForExperiment(agent);
        agent->registerAgent(actionResponse->agentId == essentials::SystemConfig::getOwnRobotID());
        agent->updateArrow(true); // FIXME
    }
    agent->updatePosition(field);
    agent->updateHeading(actionResponse->heading);
    if (responsesContain(actionResponse->responses, WumpusEnums::goldFound)) {
        agent->updateHaveGold(true);
    }

    field->updateDrafty(responsesContain(actionResponse->responses, WumpusEnums::drafty));
    field->updateShiny(responsesContain(actionResponse->responses, WumpusEnums::shiny));
    field->updateStinky(responsesContain(actionResponse->responses, WumpusEnums::stinky));
    field->updateExplored(true);
    field->updateVisited(true, agent->id);

    auto turnInfo = std::make_shared<supplementary::InformationElement<bool>>(
            actionResponse->agentId == essentials::SystemConfig::getOwnRobotID() && responsesContain(actionResponse->responses, WumpusEnums::yourTurn),
            wm->getTime(), this->turnInfoValidityDuration, 1.0);

    std::cout << "TURNINFO: " << turnInfo->getInformation();

    this->isMyTurnBuffer->add(turnInfo);

    // yourTurn message is the last message the Simulator sends
    if (actionResponse->agentId == essentials::SystemConfig::getOwnRobotID() && responsesContain(actionResponse->responses, WumpusEnums::yourTurn)) {
        this->integratedFromSimulatorForTurnNumber = true; // this->turn;
        this->raiseTurnCounter();
    }
}

void WumpusSimData::processAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception)
{
    std::lock_guard<std::mutex> lock(this->wm->resetMtx); // FIXME
    std::lock_guard<std::mutex> lock2(this->wm->planningModule->planningMtx);
    std::lock_guard<std::mutex> lock3(this->respMtx); // FIXME fix segfault rc properly

    std::cout << "Process Agent Perception: Encoding is: " << agentPerception->encoding << std::endl;
    if (!agentPerception->encoding.empty() && agentPerception->encoding != this->wm->currentEncoding) {
        std::cout << "Process Agent Perception: Encoding has been updated from: " << this->wm->currentEncoding << std::endl;

        this->wm->currentEncoding = agentPerception->encoding;
    }

    // allow communicating of dead/exited for result logging
    auto id = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
    if (this->wm->getEngine()->getRoleAssignment()->getRole(id)->getName() == this->wm->spawnRequestHandlerRoleName) {
        auto result = this->wm->experiment->getCurrentRun()->getCurrentResult();
        result->updateAgentInfo(agentPerception->senderID, std::make_pair(agentPerception->initialPosition.x, agentPerception->initialPosition.y),
                agentPerception->died, agentPerception->exited);

        if (agentPerception->exited) {
            // at least one agent needs to exit for success
            this->wm->experiment->getCurrentRun()->getCurrentResult()->exited.insert(agentPerception->senderID);
            if (this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus != eval::CompletionStatus::TIMEOUT) {
                this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus = eval::CompletionStatus::SUCCESS;
            }
            //++this->timesCompletionStatusApplied;
        }

        if (agentPerception->died) {
            this->wm->experiment->getCurrentRun()->getCurrentResult()->died.insert(agentPerception->senderID);
            if (this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus != eval::CompletionStatus::SUCCESS) {
                this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus = eval::CompletionStatus::FAILURE;
            }
            //++this->timesCompletionStatusApplied;
        }
    }

    if (this->wm->resettedForEncoding.find(agentPerception->encoding) != this->wm->resettedForEncoding.end()) {
                std::cout << "Already resetted for encoding " << agentPerception->encoding << std::endl;

        return;
    } else {
        //        std::cout << "not resetted for encoding " << agentPerception->encoding << std::endl;
    }

    if (this->wm->localAgentExited || this->wm->localAgentDied) {
        std::cout << "LOCAL AGENT EXITED/DIED!" << std::endl;

        return;
    }

    if (!this->communicationAllowed) {
        std::cout << "COMMUNIACTION NOT ALLOWED!" << std::endl;
        return;
    }

    // nothing new should come from own perception
    if (agentPerception->senderID == essentials::SystemConfig::getOwnRobotID()) {
        return;
    }
    std::cout << "Got perception from other agent! Agent exited: " << (agentPerception->exited ? "true" : "false") << std::endl;
    // already integrated info FIXME reset?
    //    if (this->integratedFromOtherAgentsForTurnNr.find(agentPerception->senderID) != this->integratedFromOtherAgentsForTurnNr.end()) {
    //        if (this->integratedFromOtherAgentsForTurnNr.at(agentPerception->senderID)) {
    //            std::cout << "Already integrated info from agent!" << std::endl;
    //            return;
    //        }
    //    }

    auto agent = this->wm->playground->getAgentById(agentPerception->senderID);
    if (!agent) {
        std::cout << "Cant access agent from model with id " << agentPerception->senderID << std::endl;
                throw std::exception(); //FIXME remove
        return;
        // TODO this might not be a good idea in case one agent already exited
        //        if (!(agentPerception->died || agentPerception->exited)) {
        //            agent = std::make_shared<wumpus::model::Agent>(this->wm->changeHandler, agentPerception->senderID);
        //            this->wm->playground->addAgent(agent);
        //            this->wm->playground->addAgentForExperiment(agent);
        //        }
    }
    std::cout << "Setting agent perception from " << agentPerception->senderID << std::endl;
    agent->updateArrow(agentPerception->arrow);
    agent->updateHaveGold(agentPerception->haveGold);
    agent->updateExhausted(agentPerception->exhausted);
    agent->updateObjective(WumpusWorldModel::objectiveFromString(agentPerception->objective));

    if (agentPerception->shot) {
        if (!agent->shot) {
            std::unordered_set<std::shared_ptr<wumpus::model::Field>> fields;
            for (auto shotAtField : agentPerception->shootingTargets) {
                fields.insert(this->wm->playground->getField(shotAtField.x, shotAtField.y));
            }
            agent->updateShotAtFields(fields);
            for (auto coordinates : agentPerception->shootingTargets) {
                this->wm->playground->addShootingTarget( // TODO still necessary?
                        agentPerception->senderID, std::make_pair<std::string, std::string>(std::to_string(coordinates.x), std::to_string(coordinates.y)));
                agent->updateShot();
                auto field = this->wm->playground->getField(coordinates.x, coordinates.y);
                field->updateShotAt(agent->id, true);
                if (agentPerception->receivedScream) {
                    for (auto f : fields) {
                        auto adj = this->wm->playground->getAdjacentFields(f->x, f->y);
                        for (auto a : adj) {
                            a->updateExplored(false);
                            for (auto i : this->wm->playground->getAgents(false)) {
                                a->updateVisited(false, i.second->id);
                            }
                        }
                    }
                }
                //        for (const auto& affected: this->wm->playground->getAdjacentFields(coordinates.x, coordinates.y)) {
                //            affected->updateStinky(false);
                //        }
            }
            std::cout << "Agent perception: shot, incrementing counter" << std::endl; // TODO
            //            this->incrementAwaitingScreamOrSilence(); // only tracked for own shots now
        }
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->replanNecessary = true;
        //        // integrate shooting targets
        //        bool found = false;
        //        auto startTime = this->wm->getEngine()->getAlicaClock()->now();
        //        long timeElapsed = 0;
        //        while (!found || timeElapsed > 3000) {
        //            timeElapsed = (this->wm->getEngine()->getAlicaClock()->now() - startTime).inMilliseconds();
        //            auto targetMap = this->wm->playground->getFieldsShotAtByAgentIds();
        //            if (targetMap->find(agent->id) != targetMap->end()) {
        //                for (const auto& target : this->wm->playground->getFieldsShotAtByAgentIds()->at(agent->id)) {
        //                    std::cout << "WumpusSimData: Integrating shooting target from other agent" << std::endl;
        //                    //                    auto field = this->wm->playground->getField(target->x), target->y);
        //                    target->updateShotAt(agentPerception->senderID, true);
        //                }
        //                found = true;
        //            }
        //        }
    }

    std::unordered_set<std::shared_ptr<wumpus::model::Field>> affectedFields;
    for (auto wumpusCoordinates : agentPerception->blockingWumpi) {
        affectedFields.insert(this->wm->playground->getField(wumpusCoordinates.x, wumpusCoordinates.y));
    }
    agent->updateBlockingWumpi(affectedFields);
    affectedFields.clear();
    for (auto trapCoordinates : agentPerception->blockingTraps) {
        affectedFields.insert(this->wm->playground->getField(trapCoordinates.x, trapCoordinates.y));
    }
    agent->updateBlockingTraps(affectedFields);
    affectedFields.clear();
    for (auto possibleNext : agentPerception->possibleNextFields) {
        affectedFields.insert(this->wm->playground->getField(possibleNext.x, possibleNext.y));
    }
    agent->updatePossibleNextFields(affectedFields);
    //    std::cout << "Processing agent perception" << std::endl;

    //    std::cout << "integrated from agents: " << std::endl;
    //    for (auto& elem : this->integratedFromOtherAgentsForTurnNr) {
    //        std::cout << elem.first << ", exited: " << agentPerception->exited << ", died: " << agentPerception->died << std::endl;
    //    }
    std::cout << "############ WUMPUSSIMDATA: UPDATE STUFF FROM AGENT PERCEPTION " << agentPerception->senderID << std::endl;

    if (this->integratedFromOtherAgentsForTurnNr.find(agentPerception->senderID) == this->integratedFromOtherAgentsForTurnNr.end()) {
        std::cout << "IFA: couldn't find" << agentPerception->senderID << std::endl;
        if (!(agentPerception->exited || agentPerception->died)) {
            std::cout << "IFA: adding " << agentPerception->senderID << std::endl;
            this->integratedFromOtherAgentsForTurnNr.emplace(agentPerception->senderID, true);
        }
    } else {
        this->integratedFromOtherAgentsForTurnNr.at(agentPerception->senderID) = true;
    }

    // remove agent if they exited or died - after this, no field info should be updated if the agent exited
    if (this->integratedFromOtherAgentsForTurnNr.find(agentPerception->senderID) != this->integratedFromOtherAgentsForTurnNr.end() &&
            (agentPerception->exited || agentPerception->died)) {
        if (agentPerception->died) {
            agent->setDead(this->wm->playground->getField(agentPerception->position.x, agentPerception->position.y));
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->replanNecessary = true;
            std::cout << "WSD: Agent Dead!" << std::endl;
            this->integratedFromOtherAgentsForTurnNr.erase(agentPerception->senderID);
            this->wm->playground->removeAgent(agent->id);
            return;
        } else if (agentPerception->exited) {
            agent->setExited();
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->replanNecessary = true;
            std::cout << "Agent " << agentPerception->senderID << " exited!" << std::endl;
            this->integratedFromOtherAgentsForTurnNr.erase(agentPerception->senderID);
            this->wm->playground->removeAgent(agent->id);
            return; // TODO special case
        }
    }

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
            field->updateVisited(true, agent->id);
            agent->updatePosition(field);
        } else {
            std::cout << "WumpusSimData: No field with given coordinates! " << std::endl;
        }
    }

    // new, possibly useful info -> reset exhausted
    //    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(false); //FIXME check if this is needed somewhere
    //    else
    // TODO new stuff - method should be refactoredc

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
    this->awaitingScreamOrSilence = 0;
    auto id = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
    this->timesCompletionStatusApplied = 0;
    this->isAwaitingShootingFeedback = false;
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

void WumpusSimData::incrementAwaitingScreamOrSilence()
{
    //    std::lock_guard<std::mutex> lock(this->awaitingMtx);
    ++this->awaitingScreamOrSilence;
    std::cout << "WSD: Awaiting scream or silence increment" << this->awaitingScreamOrSilence << std::endl;
}

int WumpusSimData::getAwaitingScreamOrSilence()
{
    //    std::lock_guard<std::mutex> lock(this->awaitingMtx);
    return this->awaitingScreamOrSilence;
}

void WumpusSimData::decrementAwaitingScreamOrSilence()
{
    //    std::lock_guard<std::mutex> lock(this->awaitingMtx);
    --this->awaitingScreamOrSilence;
    std::cout << "WSD: Awaiting scream or silence decrement " << this->awaitingScreamOrSilence << std::endl;
}

bool WumpusSimData::completionStatusApplied()
{
    std::cout << "Completion status applied: " << this->timesCompletionStatusApplied << ", " << this->wm->getPresetAgentCount() << std::endl;
    auto localId = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
    if (this->wm->getEngine()->getRoleAssignment()->getRole(localId)->getName() == this->wm->spawnRequestHandlerRoleName) {
        auto currentResult = this->wm->experiment->getCurrentRun()->getCurrentResult();
        auto addsUp = currentResult->died.size() + currentResult->exited.size();
        std::cout << "ADDSUP: " << addsUp << std::endl;
        //        return this->timesCompletionStatusApplied == this->wm->getPresetAgentCount() && addsUp;
        return addsUp;
    }
    return true;
}

bool WumpusSimData::getIsAwaitingShootingFeedback() const
{
    std::cout << "get is awaiting shooting feedback!" << std::endl;
    return isAwaitingShootingFeedback;
}

void WumpusSimData::setIsAwaitingShootingFeedback(bool isAwaitingShootingFeedback2)
{
    std::cout << "set is awaiting shooting feedback " << isAwaitingShootingFeedback2 << std::endl;
    this->isAwaitingShootingFeedback = isAwaitingShootingFeedback2;
}
}
} /* namespace wumpus */
