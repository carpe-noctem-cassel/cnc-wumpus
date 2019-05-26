#ifndef DetermineObjective_H_
#define DetermineObjective_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1554202567492) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DetermineObjective : public DomainBehaviour
    {
    public:
        DetermineObjective();
        virtual ~DetermineObjective();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1554202567492) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1554202567492) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1554202567492) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DetermineObjective_H_ */
