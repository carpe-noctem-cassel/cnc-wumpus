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
class SafePathExistsPlanner : public Planner
{
public:
    SafePathExistsPlanner(aspkb::Extractor* extractor, aspkb::Integrator* integrator);
    void checkSafePathsExistsForOtherAgents(std::pair<int, int> from, const std::map<int, std::pair<int, int>>& tos);
    void clearProblemsMap();

private:
    static std::map<std::pair<int, int>, std::map<std::pair<int, int>, std::shared_ptr<aspkb::IncrementalProblem>>> safePathExistsForOtherAgentProblems;
    aspkb::Integrator* integrator;
    int maxHorizonFactor;
};

} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */
