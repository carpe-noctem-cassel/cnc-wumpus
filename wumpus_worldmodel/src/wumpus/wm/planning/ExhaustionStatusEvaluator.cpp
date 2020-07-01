#include "wumpus/wm/planning/ExhaustionStatusEvaluator.h"
#include <wumpus/wm/util/PlannerUtils.h>
#include <aspkb/Integrator.h>
#include <wumpus/WumpusWorldModel.h>
namespace wumpus
{
namespace wm
{
namespace planning
{
ExhaustionStatusEvaluator::ExhaustionStatusEvaluator(aspkb::Extractor* extractor,aspkb::Integrator* integrator)
        : Planner(extractor), integrator(integrator)
{
    auto sc = essentials::SystemConfig::getInstance();
    auto filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("allAgentsExhaustedFilePath", NULL);
    util::PlannerUtils::loadAdditionalRules(filePath, this->allAgentsExhaustedRules);
}
bool ExhaustionStatusEvaluator::determineAllAgentsExhausted()
{
    auto res = this->extractor->extractReusableTemporaryQueryResult({"allAgentsExhausted"}, "exhaustedAgents", this->allAgentsExhaustedRules);
    if (!res.empty()) {
        this->integrator->integrateInformationAsExternal("everyAgentExhausted", "everyAgentExhausted", true, aspkb::Strategy::INSERT_TRUE);
#ifdef PM_DEBUG
        std::cout << "PlanningModule: All agents are exhausted!" << std::endl;
#endif
        return true;
    }
#ifdef PM_DEBUG
    std::cout << "PlanningModule: There are still agents who are not exhausted!" << std::endl;
#endif
    // TODO experimental
    this->integrator->integrateInformationAsExternal("everyAgentExhausted", "everyAgentExhausted", false, aspkb::Strategy::INSERT_TRUE);
    return false;
}
} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */