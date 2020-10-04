using namespace std;
#include "Plans/SingleAgent/Behaviours/PerformAction.h"

/*PROTECTED REGION ID(inccpp1534836764921) ENABLED START*/ // Add additional includes here
#include <aspkb/Extractor.h>
#include <engine/AlicaEngine.h>
#include <wumpus/model/Agent.h>
#include <wumpus/model/Field.h>
#define PERFORMACTION_DBG
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
    this->shotLastIteration = false;
    /*PROTECTED REGION END*/
}
void PerformAction::run(void* msg)
{
    /*PROTECTED REGION ID(run1534836764921) ENABLED START*/ // Add additional options here

    // wait for wm to integrate all new information into knowledge base
    //    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    auto ownId = this->sc->getOwnRobotID();

    //        if(this->wm->wumpusSimData.getIntegratedFromSimulator() && !this->wm->wumpusSimData.isIntegratedFromAllAgents()) {
    //            auto currentPos = this->wm->playground->getAgentById(ownId)->currentPosition;
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
                      !this->wm->wumpusSimData.getIsAwaitingShootingFeedback(); // &&
                                                                                //(this->wm->wumpusSimData.getAwaitingScreamOrSilence() == 0);
    if (!this->wm->wumpusSimData.isIntegratedFromAllAgents()) {
        std::cout << "PerformAction: Not integrated from all agents!" << std::endl;
    }
    //    if (this->wm->wumpusSimData.getAwaitingScreamOrSilence() != 0 || this->wm->wumpusSimData.communicationAllowed) {
    //        std::cout << "PerformAction: Awaiting scream or silence" << this->wm->wumpusSimData.getAwaitingScreamOrSilence() << std::endl;
    //    }
    if (this->wm->wumpusSimData.getIsAwaitingShootingFeedback()) {
        std::cout << "PerformAction: Is awaiting shooting feedback" << std::endl;
    }
    if (!this->wm->wumpusSimData.getIntegratedFromSimulator()) {
        std::cout << "PerformAction: Awaiting integrated from simulator" << std::endl;
    }

    if (this->shotLastIteration && !this->wm->wumpusSimData.getIsAwaitingShootingFeedback()) {
        std::cout << "Agent shot last iteration and got the required feeback: returning!" << std::endl;
        auto perception = createAgentPerception(ownId);
        send(perception);
        this->shotLastIteration = false;
        return;
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
    if (!this->shotLastIteration) { // one iteration is just for sending the complete perception
#ifdef PERFORMACTION_DBG
        std::cout << "################PERFORMACTION: START PLANNING!" << std::endl;
#endif
        this->currentActionSequence = this->wm->planningModule->processNextActionRequest(me).second;
        auto shot = this->executeNextAction();
        if (shot) {
            std::cout << "PerformAction: Agent shot, returning!" << std::endl;
            return;
        }
    }

#ifdef PERFORMACTION_DBG

    std::cout << "################PERFORMACTION: send agent perception!" << std::endl;

#endif
    auto perception = createAgentPerception(ownId);

    send(perception);

//    if (this->shotLastIteration || (!this->shotLastIteration && !this->wm->wumpusSimData.getIsAwaitingShootingFeedback())) {
    if (!this->shotLastIteration && !this->wm->wumpusSimData.getIsAwaitingShootingFeedback()) {
#ifdef PERFORMACTION_DBG
        std::cout << "PerformAction: set Success" << std::endl;
#endif
//        this->setSuccess();
        this->wm->setPerformActionSuccess(true);
        this->wm->wumpusSimData.setIntegratedFromSimulator(false);
        this->wm->wumpusSimData.resetIntegratedFromAgents();
        this->shotLastIteration = false;
    }

    /*PROTECTED REGION END*/
}

void PerformAction::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1534836764921) ENABLED START*/ // Add additional options here
    // TODO can re-entry happen?
    // this->currentActionSequence.clear();
    this->wm->setPerformActionSuccess(false);
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1534836764921) ENABLED START*/ // Add additional methods here
                                                            /**
                                                             *
                                                             * @return true if agent shot
                                                             */
bool PerformAction::executeNextAction()
{
    if (!this->currentActionSequence.empty()) {
        wumpus::model::communication::ActionRequestData req;
        auto ownId = this->sc->getOwnRobotID();
        req.agentId = ownId;
        auto action = this->currentActionSequence.begin();
        req.action = *action;
        auto result = this->wm->experiment->getCurrentRun()->getCurrentResult();
        if (result) {
            this->wm->experiment->getCurrentRun()->getCurrentResult()->increaseActionsCostCounter(
                    essentials::SystemConfig::getOwnRobotID()); // TODO rename result class?
        }

        //        std::cout << "PERFORM ACTION: " << req.action << std::endl;
        send(req);
        this->currentActionSequence.erase(action);
        if (req.action == WumpusEnums::actions::shoot) {
            this->wm->wumpusSimData.setIsAwaitingShootingFeedback(true);
            this->shotLastIteration = true;
            //            this->wm->wumpusSimData.incrementAwaitingScreamOrSilence();
            this->wm->playground->getAgentById(ownId)->updateArrow(false);
        }
        return req.action == WumpusEnums::actions::shoot;
    }
    return false;
}
/*PROTECTED REGION END*/
} /* namespace alica */
