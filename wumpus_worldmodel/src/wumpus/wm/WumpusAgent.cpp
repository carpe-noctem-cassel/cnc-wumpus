#include "wumpus/wm/WumpusAgent.h"

namespace wumpus
{
namespace wm
{

WumpusAgent::WumpusAgent(int agentId, int x, int y, int heading, bool hasArrow)
{
	this->id = agentId;
	this->initialPosition = std::make_shared<wumpus::wm::Field>(x,y);
	this->currentPosition = std::make_shared<wumpus::wm::Field>(x,y);
	this->currentHeading = heading;
	this->hasArrow = hasArrow;
}

WumpusAgent::~WumpusAgent()
{
    // TODO Auto-generated destructor stub
}

} /* namespace wm */
} /* namespace wumpus */
