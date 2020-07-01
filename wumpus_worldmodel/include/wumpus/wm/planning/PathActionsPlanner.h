#pragma once

#include "BlockageEvaluator.h"
#include "ExhaustionStatusEvaluator.h"
#include "GoalPlanner.h"
#include "ObjectiveEvaluator.h"
#include "Planner.h"

namespace aspkb
{
class Integrator;
}
namespace wumpus
{
namespace model
{
class Agent;
}
namespace wm
{
namespace planning
{
class PathActionsPlanner : public Planner
{
public:
    PathActionsPlanner(aspkb::Extractor* extractor, aspkb::Integrator* integrator);
    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> tryDetermineActionsByObjective(
            const std::shared_ptr<wumpus::model::Agent>& agent);
    GoalPlanner goalPlanner; // TODO needs to be exposed to wm

private:
    ObjectiveEvaluator objEval;

    BlockageEvaluator blockEval;
    ExhaustionStatusEvaluator exhaustionEval;

    aspkb::Integrator* integrator;

    std::shared_ptr<aspkb::IncrementalProblem> pathAndActionsProblem;

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> tryGetSafeActions(const std::shared_ptr<wumpus::model::Agent>& agent);

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> determinePathAndActions();

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> determineActionsNoMovement();

    int maxHorizonFactor;
};
}
}
}
