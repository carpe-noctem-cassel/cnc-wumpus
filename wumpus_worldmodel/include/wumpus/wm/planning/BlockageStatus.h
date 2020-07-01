#pragma once

#include <utility>

namespace wumpus
{
namespace model
{
class Field;
}
namespace wm
{

namespace planning
{
class BlockageStatus
{
public:
    BlockageStatus(std::unordered_set<std::shared_ptr<wumpus::model::Field>> blockingWumpi,
            std::unordered_set<std::shared_ptr<wumpus::model::Field>> blockingTraps, bool allPathsBlocked)
            : blockingWumpi(std::move(blockingWumpi))
            , blockingTraps(std::move(blockingTraps))
            , allPathsBlocked(allPathsBlocked)
    {
    }

    const bool allPathsBlocked; // this is more or less for debugging purposes as this should always be true if blockage is evaluated
    const std::unordered_set<std::shared_ptr<wumpus::model::Field>> blockingWumpi;
    const std::unordered_set<std::shared_ptr<wumpus::model::Field>> blockingTraps;
};
}
}
}