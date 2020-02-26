using namespace std;
#include "Plans/Reset.h"

/*PROTECTED REGION ID(inccpp1572878618173) ENABLED START*/ // Add additional includes here
#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <aspkb/Integrator.h>
#include <aspkb/TermManager.h>
#include <engine/AlicaEngine.h>
#include <reasoner/asp/Solver.h>
#include <wumpus/model/Agent.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1572878618173) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
Reset::Reset()
        : DomainBehaviour("Reset")
{
    /*PROTECTED REGION ID(con1572878618173) ENABLED START*/ // Add additional options here
    this->resetPerformed = false;
    /*PROTECTED REGION END*/
}
Reset::~Reset()
{
    /*PROTECTED REGION ID(dcon1572878618173) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void Reset::run(void* msg)
{
    /*PROTECTED REGION ID(run1572878618173) ENABLED START*/ // Add additional options here
    //        if (!this->resetPerformed)
    //        {
    //            auto agents = this->wm->playground->getAgentsForExperiment();
    //            agents->erase(essentials::SystemConfig::getOwnRobotID());
    //            for (auto a : *agents)
    //            {
    //                std::cout << "Reset: Exited" << a.second->exited << ", died:" << a.second->died << std::endl;
    //                if (!a.second->exited && !a.second->died)
    //                {
    //                    allExitedOrDied = false;
    //                }
    //            }
    //        }
    if (!this->resetPerformed) {
        wumpus_msgs::AgentPerception perception;
        wumpus_msgs::Coordinates pos;
        pos.x = -1;
        pos.y = -1;
        perception.initialPosition = pos;
        perception.position = pos;
        perception.senderID = sc->getOwnRobotID();
        perception.died = this->wm->localAgentDied;
        perception.exited = this->wm->localAgentExited; // FIXME hack
        if (this->wm->localAgentIsSpawnRequestHandler()) {
            perception.encoding = this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding();
        }
        send(perception);
        bool allExitedOrDied = true;
        //        std::cout << "In Reset!" << std::endl;

        // if not planning and all info integrated
        if (!this->wm->planningModule->getIsPlanning()) {
            std::lock_guard<std::mutex> integrating_lock(aspkb::Integrator::integratingMtx);
            std::cout << "***********Time to reset!" << std::endl;
            auto* solverWrapper = this->getEngine()->getSolver<alica::reasoner::ASPSolverWrapper>();
            solverWrapper->reset();
            this->resetPerformed = true;
            aspkb::TermManager::getInstance().clear();
            //        std::cout << "Reset: Cleared TermManager" << std::endl;
            aspkb::TermManager::getInstance().initializeSolver(solverWrapper->getSolver());
            //            std::cout << "Reset: Re-Initialized solver for TermManager!" << std::endl;
            //        std::cout << "Reset: Cleared WorldModel - re-initializing" << std::endl;
            //        this->wm->init();
            //        std::cout << "Reset: Re-Initialized WM" << std::endl;

            //            if (solverWrapper->resetted)
            //            {
            this->wm->reset();
            std::cout << "RESETTED!" << std::endl;
            this->setSuccess();
        }

        //            }
    }

    // TODO experimental...

    //    if(this->wm->localAgentDied || this->wm->localAgentExited) {
    /*PROTECTED REGION END*/
}
void Reset::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1572878618173) ENABLED START*/ // Add additional options here
                                                                             //    std::cout << "Reset:Initialize Parameters " << std::endl;
    this->resetPerformed = false;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1572878618173) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
