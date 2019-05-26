#ifndef EvaluateActions_H_
#define EvaluateActions_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1551695117454) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class EvaluateActions : public DomainBehaviour
    {
    public:
        EvaluateActions();
        virtual ~EvaluateActions();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1551695117454) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1551695117454) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1551695117454) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* EvaluateActions_H_ */
