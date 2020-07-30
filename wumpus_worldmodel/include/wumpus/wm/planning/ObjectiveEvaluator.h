#pragma once
#include "Planner.h"
#include <wumpus/model/Agent.h>

#include "SafePathExistsPlanner.h"
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
class ObjectiveEvaluator : public Planner
{
public:
    ObjectiveEvaluator(aspkb::Extractor* extractor,aspkb::Integrator* integrator);
    ~ObjectiveEvaluator();

    wumpus::model::Objective determineObjective();

    bool agentObjectiveRequiresMovement(const std::shared_ptr<wumpus::model::Agent>& agent) const;

    bool agentObjectiveRequiresGoal(const std::shared_ptr<wumpus::model::Agent>& agent) const;

    bool objectiveImpliesSimpleAction(const std::shared_ptr<wumpus::model::Agent>& agent) const;

    void clearProblemsMap();

private:
    std::vector<std::string> objectiveRules;
    SafePathExistsPlanner safePathExistsPlanner;
    aspkb::Integrator* integrator;
};
}
}
}
