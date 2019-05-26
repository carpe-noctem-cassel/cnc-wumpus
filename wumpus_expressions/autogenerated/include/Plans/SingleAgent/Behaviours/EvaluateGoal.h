#ifndef EvaluateGoal_H_
#define EvaluateGoal_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1551695082090) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class EvaluateGoal : public DomainBehaviour
    {
    public:
        EvaluateGoal();
        virtual ~EvaluateGoal();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1551695082090) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1551695082090) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1551695082090) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* EvaluateGoal_H_ */
