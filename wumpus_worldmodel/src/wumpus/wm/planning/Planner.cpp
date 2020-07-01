#include "wumpus/wm/planning/Planner.h"
#include <wumpus/WumpusWorldModel.h>
namespace wumpus
{
namespace wm
{
namespace planning
{
Planner::Planner(aspkb::Extractor* extractor)
        : extractor(extractor)
        , wm(wumpus::WumpusWorldModel::getInstance())
{
}

} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */
