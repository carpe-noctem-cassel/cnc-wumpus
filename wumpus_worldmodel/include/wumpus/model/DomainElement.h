#pragma once
#include <wumpus/wm/ChangeHandler.h>
namespace wumpus
{
namespace model
{
class DomainElement
{
public:
    DomainElement(wumpus::wm::ChangeHandler* ch)
            : ch(ch) {}

    ~DomainElement() = default;

protected:
    wumpus::wm::ChangeHandler* ch;
};
}
}
