#pragma once

#include <WumpusEnums.h>
#include <aspkb/PossibleNextFieldGenerationProblem.h>
#include <map>
#include <memory>
#include <mutex>
#include <reasoner/asp/PredicateContainer.h>
#include <reasoner/asp/SyntaxUtils.h>
#include <set>
#include <string>
#include <utility> //pair
#include <vector>
#include <wumpus/model/Objective.h>
#include <wumpus/wm/planning/BlockageEvaluator.h>
#include <wumpus/wm/planning/ExhaustionStatusEvaluator.h>
#include <wumpus/wm/planning/ObjectiveEvaluator.h>
#include <wumpus/wm/planning/PathActionsPlanner.h>

namespace aspkb
{
class IncrementalProblem;
class Extractor;
class Integrator;
}
namespace essentials
{
class SystemConfig;
}
namespace wumpus
{
class WumpusWorldModel;
namespace model
{
class Agent;
class Field;
}
namespace wm
{
class PlanningModule
{

public:
    PlanningModule(wumpus::WumpusWorldModel* wm, aspkb::Extractor* extractor, aspkb::Integrator* integrator);


    ~PlanningModule();

    std::pair<int, std::vector<WumpusEnums::actions>> processNextActionRequest(std::shared_ptr<wumpus::model::Agent> agent);

    bool getIsPlanning();
    void setIsPlanning(bool planning);

    static std::mutex planningMtx;
private:
    wumpus::WumpusWorldModel* wm;

    planning::BlockageEvaluator blockEval;
    planning::ExhaustionStatusEvaluator exhaustionEval;
    planning::ObjectiveEvaluator objEval;
    planning::PathActionsPlanner pathActionsPlanner;
    aspkb::Integrator* integrator;
    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> lastPathAndActions;

    bool isPlanning;

    bool communicationAllowed;
};
} /*namespace wm */
} /*namespace wumpus*/
