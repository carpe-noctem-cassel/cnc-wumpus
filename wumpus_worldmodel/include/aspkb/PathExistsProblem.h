#pragma once

#include "IncrementalProblem.h"
#include <reasoner/asp/Solver.h>

namespace aspkb
{
class PathExistsProblem : public IncrementalProblem
{
    static std::shared_ptr<reasoner::asp::IncrementalQueryWrapper> sharedWrapper;

public:
    static std::shared_ptr<aspkb::PathExistsProblem> get(reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
            const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename,
            const std::string& configurationSection, const std::string& externalPrefix, int startHorizon, int maxHorizon);

    // protected: FIXME why does this not work?
    PathExistsProblem(reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
            const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename,
            const std::string& configurationSection, const std::string& externalPrefix,int startHorizon, int maxHorizon,
            std::shared_ptr<reasoner::asp::IncrementalQueryWrapper> wrapper);
};
}
