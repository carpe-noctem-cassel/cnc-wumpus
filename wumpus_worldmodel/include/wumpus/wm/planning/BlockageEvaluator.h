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

    bool determineWumpusBlocksSafeMovesForSelf();
    bool determineTrapBlocksSafeMovesForSelf();

    BlockageStatus determineBlockingElementsForSelf();
private:

    std::unordered_set<std::shared_ptr<wumpus::model::Field>> generatePossibleNextFields();
    std::map<std::pair<int, int>, std::shared_ptr<aspkb::PossibleNextFieldGenerationProblem>> possibleNextFromProblems;
    aspkb::Integrator* integrator;
};
}
}
}
