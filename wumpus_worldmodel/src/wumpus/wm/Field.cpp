#include "wumpus/wm/Field.h"
namespace wumpus
{
namespace wm
{



Field::Field(Coordinates coords)
{
    this->coords = coords;
    this->perceptions = std::vector<std::string>();

}

Field::Field(int x, int y) {
	this->coords.x = x;
	this->coords.y = y;
	this->perceptions = std::vector<std::string>();
}

Field::~Field() {

}

} /* namespace wm */
} /* namespace wumpus */
