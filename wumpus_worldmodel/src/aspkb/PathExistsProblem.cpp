#include "aspkb/PathExistsProblem.h"

#include <utility>

namespace aspkb
{
// void PathExistsProblem::activateStep(int horizon)
//{
//    IncrementalProblem::activateStep(horizon);
//}
std::shared_ptr<reasoner::asp::IncrementalQueryWrapper> PathExistsProblem::sharedWrapper = nullptr;

PathExistsProblem::PathExistsProblem(reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
        const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename, const std::string& configurationSection,
        const std::string& externalPrefix, int startHorizon, int maxHorizon, std::shared_ptr<reasoner::asp::IncrementalQueryWrapper> wrapper)

        : IncrementalProblem(solver, std::move(inquiryPredicates), baseTermProgramSectionParameters, configFilename, configurationSection, startHorizon,
                  maxHorizon, false, std::move(wrapper))
{
}

std::shared_ptr<aspkb::PathExistsProblem> PathExistsProblem::get(reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
        const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename, const std::string& configurationSection,
        const std::string& externalPrefix, int startHorizon, int maxHorizon)
{
    if (PathExistsProblem::sharedWrapper) {
        return std::make_shared<aspkb::PathExistsProblem>(solver, std::move(inquiryPredicates), baseTermProgramSectionParameters, configFilename,
                configurationSection, externalPrefix, startHorizon, maxHorizon, PathExistsProblem::sharedWrapper);
    }

    auto wrapper = std::make_shared<reasoner::asp::IncrementalQueryWrapper>(solver, externalPrefix);
    PathExistsProblem::sharedWrapper = wrapper;
    return std::make_shared<aspkb::PathExistsProblem>(solver, std::move(inquiryPredicates), baseTermProgramSectionParameters, configFilename,
            configurationSection, externalPrefix, startHorizon, maxHorizon, wrapper);
}
}