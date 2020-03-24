#include "wumpus/model/Field.h"
namespace wumpus
{
namespace model
{

Field::Field(wumpus::wm::ChangeHandler* ch, int x, int y)
        : DomainElement(ch)
        , x(x)
        , y(y)
        , explored(false)
        , visited(false)
        , shiny(false)
        , drafty(false)
        , stinky(false)
        , shotAt(false)
{
}

void Field::updateShiny(bool shiny)
{
    if (this->shiny != shiny) {
        this->shiny = shiny;
        this->ch->handleSetGlitter(shared_from_this());
    }
}

void Field::updateDrafty(bool drafty)
{
    if (this->drafty != drafty) {
        this->drafty = drafty;
        this->ch->handleSetDrafty(shared_from_this());
    }
}

void Field::updateStinky(bool stinky)
{
    if (this->stinky != stinky) {
        this->stinky = stinky;
        this->ch->handleChangedStench(shared_from_this());
    }
}

void Field::updateVisited(bool visited)
{
    if (this->visited != visited) {
        this->visited = visited;
        this->ch->handleChangedVisited(shared_from_this());
    }
}

void Field::updateExplored(bool explored)
{
    if(this->explored != explored) {
        this->explored = explored;
        this->ch->handleChangedExplored(shared_from_this());
    }
}
void Field::updateShotAt(int whoShot, bool shotAt)
{
    if (this->shotAt != shotAt) {
        this->shotAt = shotAt;
        this->ch->handleChangedShotAt(whoShot, shared_from_this());
    }
}

Field::~Field() = default;

} /* namespace model */
} /* namespace wumpus */
