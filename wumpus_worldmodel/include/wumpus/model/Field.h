#pragma once
#include "DomainElement.h"
#include <memory>
#include <string>
#include <vector>

namespace wumpus
{
namespace wm
{
class ChangeHandler;
}
namespace model
{

class Field : public std::enable_shared_from_this<wumpus::model::Field>, public wumpus::model::DomainElement
{
public:
    Field(wumpus::wm::ChangeHandler* ch, int x, int y);
    virtual ~Field();
    void updateShiny(bool shiny);
    void updateDrafty(bool drafty);
    void updateStinky(bool stinky);
    void updateVisited(bool visited);
    void updateShotAt(bool shotAt);
    void updateExplored(bool explored);
    int x;
    int y;
    bool stinky;
    bool drafty;
    bool shiny;
    bool visited;
    bool explored;
    bool shotAt;
};

} /* namespace wm */
} /* namespace wumpus */
