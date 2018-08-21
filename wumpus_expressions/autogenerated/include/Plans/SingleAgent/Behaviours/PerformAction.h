#ifndef PerformAction_H_
#define PerformAction_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1534836764921) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class PerformAction : public DomainBehaviour
    {
    public:
        PerformAction();
        virtual ~PerformAction();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1534836764921) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1534836764921) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1534836764921) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PerformAction_H_ */
