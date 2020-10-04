#include "wumpus/wm/planning/ObjectiveEvaluator.h"
#include <aspkb/Integrator.h>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/model/Objective.h>
#include <wumpus/wm/util/PlannerUtils.h>
namespace wumpus
{
namespace wm
{
namespace planning
{
ObjectiveEvaluator::ObjectiveEvaluator(aspkb::Extractor* extractor, aspkb::Integrator* integrator)
        : Planner(extractor)
        , integrator(integrator)
        , safePathExistsPlanner(extractor, integrator)
{
    auto sc = essentials::SystemConfig::getInstance();
    auto filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("objectiveRulesFilePath", NULL);
    util::PlannerUtils::loadAdditionalRules(filePath, this->objectiveRules);
}

ObjectiveEvaluator::~ObjectiveEvaluator()
{
    this->safePathExistsPlanner.clearProblemsMap();
}

// TODO
/**
 * Extracts the current objective and registers it as an external along with agent Id
 * @return
 */
wumpus::model::Objective ObjectiveEvaluator::determineObjective()
{

    // go home is the final objective and the safe path should definitely exist if agent managed to get to the gold field safely
    //    if (this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective != model::Objective::GO_HOME &&
    //            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective != model::Objective::COLLECT_GOLD &&
    //            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective !=
    //                    model::Objective::IDLE) { // TODO more or less a hack -> objective might still be set to collect gold even if agent already picked
    //                                              // up gold
    //                                              // check if wumpus (still) blocks moves
    //#ifdef PM_DEBUG
    //        std::cout << "PlanningModule: Determining if wumpus blocks safe moves in order to consider it in DetermineObjective" << std::endl;
    //        std::cout << "Current objective is: " << this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective << std::endl;
    //#endif
    //
    //        this->determineWumpusBlocksSafeMoves();
    //    }

    // might trigger fetchAgent objective but is only relevant if the local agent picked up gold already
    if (this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->hasGold) {
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Own Agent has gold - checking if others need help" << std::endl;
#endif
        auto positionsOfOtherAgents = std::map<int, std::pair<int, int>>();
        auto goldField = std::pair<int, int>();
        auto allAgents = this->wm->playground->getAgents(false);
        for (const auto& agent : allAgents) {
            if (agent.first != essentials::SystemConfig::getOwnRobotID() && agent.second && agent.second->currentPosition && !agent.second->hasSafePathToGold) {
                std::cout << "PlanningModule: Adding position of other agent for safePathExistsQuery" << std::to_string(agent.second->currentPosition->x)
                          << ", " << std::to_string(agent.second->currentPosition->y) << std::endl;
                positionsOfOtherAgents.emplace(agent.first, std::make_pair(agent.second->currentPosition->x, agent.second->currentPosition->y));
            }
        }
        for (auto i = 0; i < this->wm->playground->getPlaygroundSize(); ++i) {
            for (auto j = 0; j < this->wm->playground->getPlaygroundSize(); ++j) {
                auto field = this->wm->playground->getField(i, j);
                if (field->shiny) { // TODO this is currently working because this method is only called when the agent has the gold but should be improved to
                    // work regardless
                    goldField = std::make_pair(field->x, field->y);
                }
            }
        }
//        this->safePathExistsPlanner.checkSafePathsExistsForOtherAgents(goldField, positionsOfOtherAgents);
    }

    // auto sol = this->extractor->extractTemporaryQueryResult({"objective(wildcard)"}, objectiveRules, {});
    auto sol = this->extractor->extractReusableTemporaryQueryResult({"objective(wildcard)"}, "objective", this->objectiveRules);

    // especially when the agent gets stuck in the very beginning, it might make sense to set exhausted here, but it might have unwanted side effects later
    // on...
    std::basic_string<char> result;
    if (sol.empty()) { // FIXME should probably be removed
        std::cout << "PlanningModule: Determine Objective: No solution!" << std::endl;
        //        throw std::exception();
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
        this->integrator->applyChanges();
        //        wumpus::model::Objective obj = wumpus::model::Objective::UNDEFINED;
        //        obj = this->determineObjective();
        auto sol2 = this->extractor->extractReusableTemporaryQueryResult({"objective(wildcard)"}, "objective", this->objectiveRules);
        if (sol2.empty()) {
            std::cout << "PlanningModule: Should have a posible objective by now!" << std::endl;
            if (this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->isBlockedByTrap()) {
                return wumpus::model::Objective::EXPLORE;
            } else {
                std::cout << "PlanningModule: Should have a posible objective by now! Using explore as fallback" << std::endl;
                //                throw std::exception();
                return wumpus::model::Objective::EXPLORE;
            }
            //            this->wm->playground->getAgentById(essentials::SystemConfig::getInstance()->getOwnRobotID())->updateObjective(obj);
            //            this->wm->integrateChanges();
        } else if (sol2.size() > 1) {
            std::cout << "PlanningModule: Found more than one possible objective but should be well-defined 2" << std::endl;
            throw std::exception();
        } else {
            result = sol2.at(0);
        }
    }
    //        return obj;
    //        return wumpus::model::Objective::EXPLORE;
    else if (sol.size() > 1) {
        std::cout << "PlanningModule: Found more than one possible objective but should be well-defined" << std::endl;
        throw std::exception();
    } else {
        result = sol.at(0);
    }

    wumpus::model::Objective obj = wumpus::model::Objective::UNDEFINED;

#ifdef PM_DEBUG
    std::cout << "PlanningModule: DETERMINE OBJECTIVE RESULT: " << result << std::endl;
#endif

    if (result.find("explore") != std::string::npos) {
        obj = wumpus::model::Objective::EXPLORE;
        // FIXME might be problematic here...
        if (this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective != obj) {
#ifdef PM_DEBUG
            std::cout << "PlanningModule: objective changed -> setting unsafe moves allowed to false" << std::endl;
#endif
            this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", false, aspkb::Strategy::FALSIFY_OLD_VALUES);
            this->integrator->applyChanges();
        }

    } else if (result.find("huntWumpus") != std::string::npos) {
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(false);
        obj = wumpus::model::Objective::HUNT_WUMPUS;
    } else if (result.find("collectGold") != std::string::npos) {
        obj = wumpus::model::Objective::COLLECT_GOLD;
    } else if (result.find("goHome") != std::string::npos) {
        obj = wumpus::model::Objective::GO_HOME;
    } else if (result.find("shoot") != std::string::npos) {
        obj = wumpus::model::Objective::SHOOT;
    } else if (result.find("leave") != std::string::npos) {
        obj = wumpus::model::Objective::LEAVE;
    } else if (result.find("moveToGoldField") != std::string::npos) {
        obj = wumpus::model::Objective::MOVE_TO_GOLD_FIELD;
    } else if (result.find("fetchOtherAgent") != std::string::npos) {
        // FIXME remove
        //        std::cout << "PlanningModule: Wants to fetch other agent but safe path should exist" << std::endl;
        //        throw std::exception();
        obj = wumpus::model::Objective::FETCH_OTHER_AGENT;
    } else if (result.find("idle") != std::string::npos) {
        auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());
        if (!localAgent->hasGold) {
            std::cout << "LOCAL AGENT DOESNT HAVE GOLD: UPDATE EXHAUSTED" << std::endl;
            localAgent->updateExhausted(true);
        }
        obj = wumpus::model::Objective::IDLE;
    }

    //        std::cout << "OBJECTIVE: " << obj << std::endl;

    if (obj == wumpus::model::Objective::EXPLORE &&
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective == wumpus::model::Objective::HUNT_WUMPUS) {
        this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        this->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    }
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateObjective(obj);
    this->integrator->applyChanges();

    return obj;
    // return wumpus::model::Objective::EXPLORE;
}

bool ObjectiveEvaluator::objectiveImpliesSimpleAction(const std::shared_ptr<model::Agent>& agent) const
{
    return agent->objective == model::COLLECT_GOLD || agent->objective == model::SHOOT || agent->objective == model::LEAVE;
}

bool ObjectiveEvaluator::agentObjectiveRequiresGoal(const std::shared_ptr<wumpus::model::Agent>& agent) const
{
    return agent->objective == model::MOVE_TO_GOLD_FIELD || agent->objective == model::GO_HOME || agent->objective == model::Objective::FETCH_OTHER_AGENT; // ||
    //           agent->objective == model::Objective::HUNT_WUMPUS; // requires determinePosToShootFrom
}

bool ObjectiveEvaluator::agentObjectiveRequiresMovement(const std::shared_ptr<wumpus::model::Agent>& agent) const
{
    return (agent->objective == model::GO_HOME || agent->objective == model::EXPLORE || agent->objective == model::MOVE_TO_GOLD_FIELD ||
            agent->objective == model::Objective::FETCH_OTHER_AGENT || agent->objective == model::Objective::HUNT_WUMPUS);
}

} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */
