#pragma once

#include <aspkb/Extractor.h>
namespace wumpus
{
class WumpusWorldModel;
namespace wm
{
namespace planning
{
class Planner
{

public:
    explicit Planner(aspkb::Extractor* extractor);

protected:
    aspkb::Extractor* extractor;
    wumpus::WumpusWorldModel* wm;
    const char* const KB_CONFIG_NAME = "KnowledgeManager";
};
} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */
