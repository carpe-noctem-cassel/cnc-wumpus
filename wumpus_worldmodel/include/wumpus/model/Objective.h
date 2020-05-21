#pragma once

#include <ostream>

namespace wumpus
{
namespace model
{
enum Objective
{
    UNDEFINED,
    EXPLORE,
    MOVE_TO_GOLD_FIELD,
    COLLECT_GOLD,
    HUNT_WUMPUS,
    FETCH_OTHER_AGENT,
    SHOOT,
    GO_HOME,
    LEAVE,
    IDLE
};

    inline std::ostream& operator<<(std::ostream& str, Objective objective)
    {
        switch (objective) {
            case UNDEFINED:
                return str << "undefined";
            case EXPLORE:
                return str << "explore";
            case MOVE_TO_GOLD_FIELD:
                return str << "moveToGoldField";
            case COLLECT_GOLD:
                return str << "collectGold";
            case HUNT_WUMPUS:
                return str << "huntWumpus";
            case SHOOT:
                return str << "shoot";
            case GO_HOME:
                return str << "goHome";
            case LEAVE:
                return str << "leave";
            case IDLE:
                return str << "idle";
            case FETCH_OTHER_AGENT:
                return str << "fetchOtherAgent";
            default:
                return str << (int) objective;
        }
    }
}
}