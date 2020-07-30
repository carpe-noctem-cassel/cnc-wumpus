#pragma once

#include "Planner.h"
#include "BlockageEvaluator.h"

namespace aspkb
{
class Integrator;
}
namespace wumpus
{
namespace wm
{
namespace planning
{
class GoalPlanner : public Planner
{
public:
    GoalPlanner(aspkb::Extractor* extractor, aspkb::Integrator* integrator);
    std::string determineGoal();
    std::pair<std::pair<std::string, std::string>, std::string> determinePosToShootFrom();

private:
    std::vector<std::string> goalGenerationRules;
    std::vector<std::string> shootFromRules;

    BlockageEvaluator blockEval;
//    std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> shootingTargets; // FIXME
    aspkb::Integrator* integrator;
//    void addShootingTarget(int id, const std::pair<std::basic_string<char>, std::basic_string<char>>& shotAt);
//    std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> getShootingTargets();

};
}
}
}
