using namespace std;
#include "Plans/SingleAgent/Behaviours/WaitForTurn.h"

/*PROTECTED REGION ID(inccpp1534835364093) ENABLED START*/ // Add additional includes here
#include <wumpus/model/Agent.h>
#include <wumpus/model/Field.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1534835364093) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
WaitForTurn::WaitForTurn()
        : DomainBehaviour("WaitForTurn")
{
    /*PROTECTED REGION ID(con1534835364093) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
WaitForTurn::~WaitForTurn()
{
    /*PROTECTED REGION ID(dcon1534835364093) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void WaitForTurn::run(void* msg)
{
    /*PROTECTED REGION ID(run1534835364093) ENABLED START*/ // Add additional options here
    // TODO experimental...
    if (this->wm->localAgentDied || this->wm->localAgentExited) {
        auto perception = this->createAgentPerception(essentials::SystemConfig::getOwnRobotID());
        send(perception);
    }
    /*PROTECTED REGION END*/
}
void WaitForTurn::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1534835364093) ENABLED START*/ // Add additional options here
    //        std::cout << "WaitForTurn: waiting for turn" << std::endl;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1534835364093) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
