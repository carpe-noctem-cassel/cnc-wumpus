#include "wumpus/WumpusWorldModel.h"

#include <engine/AlicaEngine.h>

namespace wumpus
{

WumpusWorldModel *WumpusWorldModel::getInstance()
{
    static WumpusWorldModel instance;
    return &instance;
}

WumpusWorldModel::WumpusWorldModel() :
		WorldModel()
    , wumpusSimData(this)
	, knowledgeManager(this)
	, playground(this)
	, communication(nullptr)
{
	this->agentName = sc->getHostname();
}

WumpusWorldModel::~WumpusWorldModel()
{
	delete this->communication;
}

std::string WumpusWorldModel::getAgentName()
{
	return this->agentName;
}

void WumpusWorldModel::init()
{
	this->communication = new wm::Communication(this);

}
} /* namespace wumpus */

