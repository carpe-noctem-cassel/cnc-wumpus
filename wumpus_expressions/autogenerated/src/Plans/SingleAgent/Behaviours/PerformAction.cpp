using namespace std;
#include "Plans/SingleAgent/Behaviours/PerformAction.h"

/*PROTECTED REGION ID(inccpp1534836764921) ENABLED START*/ // Add additional includes here
#include <engine/AlicaEngine.h>
using std::make_shared;
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1534836764921) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
PerformAction::PerformAction()
    : DomainBehaviour("PerformAction")
{
    /*PROTECTED REGION ID(con1534836764921) ENABLED START*/ // Add additional options here
    this->result = std::vector<WumpusEnums::actions>();
    /*PROTECTED REGION END*/
}
PerformAction::~PerformAction()
{
    /*PROTECTED REGION ID(dcon1534836764921) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void PerformAction::run(void *msg)
{
    /*PROTECTED REGION ID(run1534836764921) ENABLED START*/ // Add additional options here
    wumpus_simulator::ActionRequest req;
    req.agentId = this->sc->getOwnRobotID();
    result = this->wm->knowledgeManager.getNextAction();
    auto action = result.at(0);
    req.action = action;
    send(req);

    //		send(actionRequest);

    /*PROTECTED REGION END*/
}
void PerformAction::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1534836764921) ENABLED START*/ // Add additional options here
    result.clear();
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1534836764921) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
