#ifndef PerformNextAction_H_
#define PerformNextAction_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1551695143200) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class PerformNextAction : public DomainBehaviour
    {
    public:
        PerformNextAction();
        virtual ~PerformNextAction();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1551695143200) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1551695143200) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1551695143200) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PerformNextAction_H_ */
