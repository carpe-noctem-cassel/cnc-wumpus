#pragma once
namespace wumpus
{
namespace wm
{

class Coordinates
{
  public:
	Coordinates(const Coordinates &c);
    Coordinates(int x, int y);
    Coordinates();
    virtual ~Coordinates();
    int x;
    int y;

    bool operator<(const Coordinates& c) const
    {
      return this->x < c.x && this->y < c.y;
    }

    bool operator==(const Coordinates& c) const
    {
      return this->x == c.x && this->y == c.y;
    }



};

} /* namespace wm */
} /* namespace wumpus */

