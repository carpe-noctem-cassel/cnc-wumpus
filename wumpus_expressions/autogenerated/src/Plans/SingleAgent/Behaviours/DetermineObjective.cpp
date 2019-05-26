using namespace std;
#include "Plans/SingleAgent/Behaviours/DetermineObjective.h"

/*PROTECTED REGION ID(inccpp1554202567492) ENABLED START*/ //Add additional includes here
#include <chrono>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1554202567492) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DetermineObjective::DetermineObjective() :
            DomainBehaviour("DetermineObjective")
    {
        /*PROTECTED REGION ID(con1554202567492) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DetermineObjective::~DetermineObjective()
    {
        /*PROTECTED REGION ID(dcon1554202567492) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DetermineObjective::run(void* msg)
    {
        /*PROTECTED REGION ID(run1554202567492) ENABLED START*/ //Add additional options here
//        if (!this->wm->wumpusSimData.getIntegrated())
//        {
//            return;
//        }
        //this->wm->planningModule->determineObjective();
        this->setSuccess();

        /*PROTECTED REGION END*/
    }
    void DetermineObjective::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1554202567492) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1554202567492) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
