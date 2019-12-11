#ifndef Reset_H_
#define Reset_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1572878618173) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class Reset : public DomainBehaviour
    {
    public:
        Reset();
        virtual ~Reset();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1572878618173) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1572878618173) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1572878618173) ENABLED START*/ //Add additional private methods here
        bool resetPerformed;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Reset_H_ */
