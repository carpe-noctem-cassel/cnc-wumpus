#pragma once

#include "Planner.h"
namespace aspkb {
    class Integrator;
}
namespace wumpus
{
namespace wm
{
namespace planning
{
class ExhaustionStatusEvaluator : public Planner
{
public:
    ExhaustionStatusEvaluator(aspkb::Extractor *extractor, aspkb::Integrator* integrator);
    bool determineAllAgentsExhausted();

private:
    std::vector<std::string> allAgentsExhaustedRules;
    aspkb::Integrator* integrator;
};
} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */
