#pragma once

//#include <aspkb/PossibleNextFieldGenerationProblem.h>
#include "Planner.h"
#include "BlockageStatus.h"

namespace aspkb {
    class Integrator;
    class PossibleNextFieldGenerationProblem;
}
namespace wumpus
{
namespace wm
{
namespace planning
{
class BlockageEvaluator : public Planner
{
public:
    BlockageEvaluator(aspkb::Extractor* extractor, aspkb::Integrator* integrator);
    ~BlockageEvaluator();

    bool determineWumpusBlocksSafeMovesForSelf();
    bool determineTrapBlocksSafeMovesForSelf();

    BlockageStatus determineBlockingElementsForSelf();
    std::unordered_set<std::shared_ptr<wumpus::model::Field>> generatePossibleNextFields();
    std::unordered_set<std::shared_ptr<wumpus::model::Field>> generatePossibleNextFieldsAlternative();

private:

    static std::map<std::pair<int, int>, std::shared_ptr<aspkb::PossibleNextFieldGenerationProblem>> possibleNextFromProblems;
    aspkb::Integrator* integrator;
    std::vector<std::string> possibleNextAlternativeRules;
};
}
}
}
