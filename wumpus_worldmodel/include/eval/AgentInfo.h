#include <utility>

#pragma once

namespace eval
{

class AgentInfo
{
public:
    AgentInfo(int id, std::pair<int, int> initialPos, bool died, bool exited)
            : id(id)
            , initialPos(std::move(initialPos))
            , died(died)
            , exited(exited)
    {
    }

    const int id;
    std::pair<int, int> initialPos;
    bool died;
    bool exited;
};
}
