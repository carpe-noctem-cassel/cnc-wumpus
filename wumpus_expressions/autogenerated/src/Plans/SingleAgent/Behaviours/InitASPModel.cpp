using namespace std;
#include "Plans/SingleAgent/Behaviours/InitASPModel.h"

/*PROTECTED REGION ID(inccpp1536061745720) ENABLED START*/ //Add additional includes here
#include <engine/AlicaEngine.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1536061745720) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    InitASPModel::InitASPModel() :
            DomainBehaviour("InitASPModel")
    {
        /*PROTECTED REGION ID(con1536061745720) ENABLED START*/ //Add additional options here
        this->query = make_shared<alica::Query>();
        /*PROTECTED REGION END*/
    }
    InitASPModel::~InitASPModel()
    {
        /*PROTECTED REGION ID(dcon1536061745720) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void InitASPModel::run(void* msg)
    {
        /*PROTECTED REGION ID(run1536061745720) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void InitASPModel::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1536061745720) ENABLED START*/ //Add additional options here
        query->clearStaticVariables();
        query->addStaticVariable(getVariable("ModelVar"));
        result.clear();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1536061745720) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
