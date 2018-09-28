#pragma once
#include <vector>
#include <string>
#include "wumpus/wm/Coordinates.h"
#include "wumpus/wm/PerceptionContainer.h"
namespace wumpus
{
namespace wm
{
class Field
{
  public:
    Field(wumpus::wm::Coordinates coords);
    Field(int x, int y);
    virtual ~Field();
    wumpus::wm::Coordinates coords;
    std::vector<std::string> perceptions;
};

} /* namespace wm */
} /* namespace wumpus */

