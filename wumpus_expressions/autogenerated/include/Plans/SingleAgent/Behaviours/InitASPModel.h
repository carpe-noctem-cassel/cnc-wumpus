#ifndef InitASPModel_H_
#define InitASPModel_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1536061745720) ENABLED START*/ //Add additional includes here
#include <engine/constraintmodul/Query.h>
/*PROTECTED REGION END*/
namespace alica
{
    class InitASPModel : public DomainBehaviour
    {
    public:
        InitASPModel();
        virtual ~InitASPModel();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1536061745720) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1536061745720) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1536061745720) ENABLED START*/ //Add additional private methods here
        shared_ptr<alica::Query> query;
        std::vector<void*> result;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* InitASPModel_H_ */
