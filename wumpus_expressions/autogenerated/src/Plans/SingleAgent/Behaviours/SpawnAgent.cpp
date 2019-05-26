using namespace std;
#include "Plans/SingleAgent/Behaviours/SpawnAgent.h"

/*PROTECTED REGION ID(inccpp1534835348868) ENABLED START*/ //Add additional includes here
#include "wumpus_simulator/InitialPoseRequest.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1534835348868) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    SpawnAgent::SpawnAgent() :
            DomainBehaviour("SpawnAgent")
    {
        /*PROTECTED REGION ID(con1534835348868) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    SpawnAgent::~SpawnAgent()
    {
        /*PROTECTED REGION ID(dcon1534835348868) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void SpawnAgent::run(void* msg)
    {
        /*PROTECTED REGION ID(run1534835348868) ENABLED START*/ //Add additional options here
        if (this->wm->localAgentDied)
        {
            return;
        }
        wumpus_simulator::InitialPoseRequest req;
        req.agentId = this->sc->getOwnRobotID();
        send(req);
        /*PROTECTED REGION END*/
    }
    void SpawnAgent::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1534835348868) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1534835348868) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
