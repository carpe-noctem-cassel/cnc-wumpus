using namespace std;
#include "Plans/SingleAgent/Behaviours/WaitForTurn.h"

/*PROTECTED REGION ID(inccpp1534835364093) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1534835364093) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    WaitForTurn::WaitForTurn() :
            DomainBehaviour("WaitForTurn")
    {
        /*PROTECTED REGION ID(con1534835364093) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    WaitForTurn::~WaitForTurn()
    {
        /*PROTECTED REGION ID(dcon1534835364093) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void WaitForTurn::run(void* msg)
    {
        /*PROTECTED REGION ID(run1534835364093) ENABLED START*/ //Add additional options here
        // nothing to do here yet
        //TODO experimental...
        if (this->wm->localAgentDied || this->wm->localAgentExited)
        {
            wumpus_msgs::AgentPerception perception;
            wumpus_msgs::Coordinates pos;
            pos.x = -1;
            pos.y = -1;
            perception.initialPosition = pos;
            perception.position = pos;
            perception.senderID = sc->getOwnRobotID();
            perception.died = this->wm->localAgentDied;
            perception.exited = this->wm->localAgentExited;
            if (this->wm->localAgentIsSpawnRequestHandler()) {
                perception.encoding = this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding();
            }
            send(perception);

        }
        /*PROTECTED REGION END*/
    }
    void WaitForTurn::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1534835364093) ENABLED START*/ //Add additional options here
//        std::cout << "WaitForTurn: waiting for turn" << std::endl;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1534835364093) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
