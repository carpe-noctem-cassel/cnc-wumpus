#pragma once

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
    SHOOT,
    GO_HOME,
    LEAVE,
    IDLE
};
}
}