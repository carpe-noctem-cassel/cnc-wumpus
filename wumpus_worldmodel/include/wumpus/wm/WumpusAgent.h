#pragma once
#include "wumpus/wm/Field.h"
#include <WumpusEnums.h>
#include <memory>
namespace wumpus
{
namespace wm
{
class Field;
class WumpusAgent
{
  public:
    WumpusAgent(int agentId, int x, int y, int heading, bool hasArrow);
    virtual ~WumpusAgent();
    std::shared_ptr<wumpus::wm::Field> initialPosition;
    std::shared_ptr<wumpus::wm::Field> currentPosition;
    int currentHeading;
    bool hasArrow;
    int id;
    std::vector<std::string> perceptions;
};

} /* namespace wm */
} /* namespace wumpus */

