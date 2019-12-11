#ifndef LogPreviousResults_H_
#define LogPreviousResults_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1575467895625) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class LogPreviousResults : public DomainBehaviour
    {
    public:
        LogPreviousResults();
        virtual ~LogPreviousResults();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1575467895625) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1575467895625) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1575467895625) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* LogPreviousResults_H_ */
