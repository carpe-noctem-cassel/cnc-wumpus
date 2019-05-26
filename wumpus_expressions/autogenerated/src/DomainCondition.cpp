#include "DomainCondition.h"
#include <SystemConfig.h>
#include <wumpus/WumpusWorldModel.h>

namespace alica
{
    DomainCondition::DomainCondition() :
            BasicCondition()
    {

    	this->wm = wumpus::WumpusWorldModel::getInstance();
    	this->sc = essentials::SystemConfig::getInstance();
    }

    DomainCondition::~DomainCondition()
    {
    }
} /* namespace alica */
