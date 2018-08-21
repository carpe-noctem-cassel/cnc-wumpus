#include "DomainCondition.h"
#include <SystemConfig.h>
#include <wumpus/WumpusWorldModel.h>

namespace alica
{
    DomainCondition::DomainCondition() :
            BasicCondition()
    {

    	this->wm = wumpus::WumpusWorldModel::getInstance();
    	this->sc = supplementary::SystemConfig::getInstance();
    }

    DomainCondition::~DomainCondition()
    {
    }
} /* namespace alica */
