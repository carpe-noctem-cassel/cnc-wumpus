#include "wumpus/model/Field.h"
namespace wumpus
{
namespace model
{

Field::Field(wumpus::wm::ChangeHandler* ch, int x, int y)
        : DomainElement(ch)
{
    this->x = x;
    this->y = y;
    this->visited = false;
    this->shiny = false;
    this->drafty = false;
    this->stinky = false;
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

void Field::updateVisited(bool visited) {
    if(this->visited != visited) {
        this->visited = visited;
        this->ch->handleChangedVisited(shared_from_this());
    }
}

Field::~Field() = default;

} /* namespace model */
} /* namespace wumpus */
