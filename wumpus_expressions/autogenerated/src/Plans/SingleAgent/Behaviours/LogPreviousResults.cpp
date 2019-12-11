using namespace std;
#include "Plans/SingleAgent/Behaviours/LogPreviousResults.h"

/*PROTECTED REGION ID(inccpp1575467895625) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1575467895625) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    LogPreviousResults::LogPreviousResults() :
            DomainBehaviour("LogPreviousResults")
    {
        /*PROTECTED REGION ID(con1575467895625) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    LogPreviousResults::~LogPreviousResults()
    {
        /*PROTECTED REGION ID(dcon1575467895625) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void LogPreviousResults::run(void* msg)
    {
        /*PROTECTED REGION ID(run1575467895625) ENABLED START*/ //Add additional options here
        this->setSuccess();
        this->setSuccess();
        /*PROTECTED REGION END*/
    }
    void LogPreviousResults::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1575467895625) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1575467895625) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
