#ifndef WaitForTurn_H_
#define WaitForTurn_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1534835364093) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class WaitForTurn : public DomainBehaviour
    {
    public:
        WaitForTurn();
        virtual ~WaitForTurn();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1534835364093) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1534835364093) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1534835364093) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* WaitForTurn_H_ */
