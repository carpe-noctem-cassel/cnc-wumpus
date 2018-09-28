using namespace std;
#include "Plans/SingleAgent/Behaviours/InitASPModel.h"

/*PROTECTED REGION ID(inccpp1536061745720) ENABLED START*/ //Add additional includes here
#include <engine/AlicaEngine.h>
#include <asp_commons/ASPQuery.h>
#include <asp_solver_wrapper/ASPSolverWrapper.h>
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
        if (this->isSuccess())
        {
            return;
        }

        query->getSolution<alica::reasoner::ASPSolverWrapper, ::reasoner::AnnotatedValVec>(runningPlan, result);

        if (result.size() > 0)
        {
            cout << "InitASPModel: Found Result!" << endl;
            cout << "Size of result vector: " << result.size() << endl;
            for (auto res : result)
            {
                cout << res.query->toString() << endl;
                for (int i = 0; i < res.variableQueryValues.size(); ++i)
                {
                    for (int j = 0; j < res.variableQueryValues.at(i).size(); ++j)
                    {
                        cout << "INIT MODEL: " << i << "," << j << ", " << res.variableQueryValues.at(i).at(j) << endl;
                    }

                }
            }
            this->setSuccess(true);

        }
        else
        {
            cout << "InitASPModel: no result found!!!" << endl;
        }
        /*PROTECTED REGION END*/
    }
    void InitASPModel::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1536061745720) ENABLED START*/ //Add additional options here
        query->clearStaticVariables();
        query->addStaticVariable(getVariableByName("ModelVar"));
        result.clear();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1536061745720) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
