#include "wumpus/wm/Coordinates.h"

namespace wumpus
{
namespace wm
{

Coordinates::Coordinates()
{
	this->x = 0;
	this->y = 0;
}

Coordinates::Coordinates(const Coordinates &c)
{
    this->x = c.x;
    this->y = c.y;
}

Coordinates::Coordinates(int x, int y)
{
    this->x = x;
    this->y = y;
}

Coordinates::~Coordinates()
{
}

} /* namespace wm */
} /* namespace wumpus */
