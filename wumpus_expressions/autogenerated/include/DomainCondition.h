#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicCondition.h"

namespace wumpus {
	class WumpusWorldModel;
}
namespace essentials {
	class SystemConfig;
}
namespace alica
{
    class DomainCondition : public BasicCondition
    {
    public:
        DomainCondition();
        virtual ~DomainCondition();

        wumpus::WumpusWorldModel* wm;
        essentials::SystemConfig* sc;
    };
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

