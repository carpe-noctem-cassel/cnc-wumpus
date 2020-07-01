#pragma once

#include "IncrementalProblem.h"
#include <reasoner/asp/Solver.h>
#include <wumpus/model/Field.h>
#include <vector>

namespace aspkb
{
class PossibleNextFieldGenerationProblem : public IncrementalProblem
{

public:
    PossibleNextFieldGenerationProblem(::reasoner::asp::Solver* solver, const std::map<std::string, std::string>& baseTermProgramSectionParameters,
            const std::string& externalPrefix, int maxHorizon);

    std::vector<std::string> doIncrementalSolving();

    void deactivateStep(int i);

    void activateInquiries(std::vector<::reasoner::asp::Term *> &terms, int horizon);
};
}
