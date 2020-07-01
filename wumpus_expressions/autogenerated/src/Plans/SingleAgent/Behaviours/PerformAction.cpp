using namespace std;
#include "Plans/SingleAgent/Behaviours/PerformAction.h"

/*PROTECTED REGION ID(inccpp1534836764921) ENABLED START*/ // Add additional includes here
#include <aspkb/Extractor.h>
#include <engine/AlicaEngine.h>
#include <wumpus/model/Agent.h>
#include <wumpus/model/Field.h>
#include <wumpus_msgs/Coordinates.h>
//#define PERFORMACTION_DBG
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1534836764921) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
PerformAction::PerformAction()
        : DomainBehaviour("PerformAction")
{
    /*PROTECTED REGION ID(con1534836764921) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
PerformAction::~PerformAction()
{
    /*PROTECTED REGION ID(dcon1534836764921) ENABLED START*/ // Add additional options here
    this->lastInformationTime = alica::AlicaTime::zero();
    this->planningQueryId = -1;
    /*PROTECTED REGION END*/
}
void PerformAction::run(void* msg)
{
    /*PROTECTED REGION ID(run1534836764921) ENABLED START*/ // Add additional options here
    // wait for wm to integrate all new information into knowledge base
    auto ownId = this->sc->getOwnRobotID();

    //        if(this->wm->wumpusSimData.getIntegratedFromSimulator() && !this->wm->wumpusSimData.isIntegratedFromAllAgents()) {
    //            auto currentPos = this->wm->playground->getAgentById(ownId)->currentPosition;
    //            wumpus_msgs::AgentPerception perception;
    //            wumpus_msgs::Coordinates coords;
    //            coords.x = currentPos->x;
    //            coords.y = currentPos->y;
    //            perception.position = coords;
    //            perception.stinky = currentPos->stinky;
    //            perception.glitter = currentPos->shiny;
    //            perception.drafty = currentPos->drafty;
    //            perception.senderID = ownId;
    //            perception.exited = this->wm->localAgentExited;
    //            perception.died = this->wm->localAgentDied;
    //            send(perception);
    //            return;
    //        }
    auto integrated = this->wm->wumpusSimData.getIntegratedFromSimulator() && this->wm->wumpusSimData.isIntegratedFromAllAgents() &&
                      !this->wm->wumpusSimData.getAwaitingScreamOrSilence();
    if (!this->wm->wumpusSimData.isIntegratedFromAllAgents()) {
        std::cout << "PerformAction: Not integrated from all agents!" << std::endl;
    }
    if (this->wm->wumpusSimData.getAwaitingScreamOrSilence()) {
        std::cout << "PerformAction: Awaiting scream or silence" << std::endl;
    }
    if (!integrated) {
#ifdef PERFORMACTION_DBG
        std::cout << "PerformAction: Didn't integrate all sensory information yet!" << std::endl;
#endif
        return;
    }
    auto me = this->wm->playground->getAgentById(ownId);

    if (!me) {
#ifdef PERFORMACTION_DBG
        std::cout << "PerformAction: Can't access own agent!" << std::endl;
#endif
        return;
    }

    // this->wm->planningModule->determineObjective();

    // TODO move action sequence into agent and publish?

    // TODO remove
    // sleep(0.3);
    std::cout << "################PERFORMACTION: START PLANNING!" << std::endl;
    this->currentActionSequence = this->wm->planningModule->processNextActionRequest(me).second;
    this->executeNextAction();
    // if (this->wm->wumpusSimData.getIntegratedFromSimulator()) {
    auto localAgent = this->wm->playground->getAgentById(ownId);
    auto currentPos = localAgent->currentPosition;
    wumpus_msgs::AgentPerception perception;
    wumpus_msgs::Coordinates coords;
    coords.x = currentPos->x;
    coords.y = currentPos->y;
    perception.position = coords;
    if (this->wm->localAgentIsSpawnRequestHandler()) {
        perception.encoding = this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding();
    }
    perception.stinky = currentPos->stinky;
    perception.glitter = currentPos->shiny;
    perception.drafty = currentPos->drafty;
    perception.senderID = ownId;
    perception.exited = this->wm->localAgentExited;
    perception.died = this->wm->localAgentDied;
    perception.haveGold = localAgent->hasGold;
    perception.exhausted = localAgent->exhausted;
    std::stringstream obj;
    obj << localAgent->objective;
    perception.objective = obj.str();
    std::cout << "PerformAction: set exhausted " << localAgent->exhausted << std::endl;
    perception.shot = localAgent->shot;
    auto shotAtFields = this->wm->playground->getFieldsShotAtByAgentIds();
    if (shotAtFields->find(essentials::SystemConfig::getOwnRobotID()) != shotAtFields->end()) {
        for (const auto& field : shotAtFields->at(essentials::SystemConfig::getOwnRobotID())) {
            wumpus_msgs::Coordinates coordinates;
            coordinates.x = field->x;
            coordinates.y = field->y;
            std::cout << "Perform Action: Agent " << essentials::SystemConfig::getOwnRobotID() << "shot at " << coordinates.x << ", " << coordinates.y
                      << std::endl;
            perception.shootingTargets.push_back(coordinates);
        }
    }
    for (const auto& blockingWumpus : localAgent->fieldsWithBlockingWumpi) {
        wumpus_msgs::Coordinates coordinates;
        coordinates.x = blockingWumpus->x;
        coordinates.y = blockingWumpus->y;
        perception.blockingWumpi.emplace_back(coordinates);
    }
    for (const auto& blockingTrap : localAgent->fieldsWithBlockingTraps) {
        wumpus_msgs::Coordinates coordinates;
        coordinates.x = blockingTrap->x;
        coordinates.y = blockingTrap->y;
        perception.blockingWumpi.emplace_back(coordinates);
    }

    send(perception);
    //            return;
    //}
    this->wm->wumpusSimData.setIntegratedFromSimulator(false);
    this->wm->wumpusSimData.resetIntegratedFromAgents();

    this->setSuccess();
    /*PROTECTED REGION END*/
}
void PerformAction::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1534836764921) ENABLED START*/ // Add additional options here
    // TODO can re-entry happen?
    // this->currentActionSequence.clear();
    this->lastInformationTime = alica::AlicaTime::zero();
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1534836764921) ENABLED START*/ // Add additional methods here
void PerformAction::executeNextAction()
{
    if (!this->currentActionSequence.empty()) {
        wumpus_simulator::ActionRequest req;
        auto ownId = this->sc->getOwnRobotID();
        req.agentId = ownId;
        auto action = this->currentActionSequence.begin();
        req.action = *action;
        auto result = this->wm->experiment->getCurrentRun()->getCurrentResult();
        if (result) {
            this->wm->experiment->getCurrentRun()->getCurrentResult()->increaseActionsCostCounter(
                    essentials::SystemConfig::getOwnRobotID()); // TODO rename result class?
        }

        std::cout << "PERFORM ACTION: " << req.action << std::endl;
        send(req);
        this->currentActionSequence.erase(action);
        if (req.action == WumpusEnums::actions::shoot) {
            this->wm->playground->getAgentById(ownId)->updateArrow(false);
        }
    }
}
/*PROTECTED REGION END*/
} /* namespace alica */
