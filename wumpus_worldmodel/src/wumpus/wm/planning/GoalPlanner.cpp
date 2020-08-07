//
// Created by lisa on 26.06.20.
//

#include "wumpus/wm/planning/GoalPlanner.h"
#include <aspkb/Integrator.h>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/wm/util/PlannerUtils.h>
namespace wumpus
{
namespace wm
{
namespace planning
{
GoalPlanner::GoalPlanner(aspkb::Extractor* extractor, aspkb::Integrator* integrator)
        : Planner(extractor)
        , integrator(integrator)
        , blockEval(extractor, integrator)
{
    auto sc = essentials::SystemConfig::getInstance();
    auto filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("goalGenerationRulesFilePath", NULL);
    util::PlannerUtils::loadAdditionalRules(filePath, this->goalGenerationRules);
    filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("shootFromRulesFilePath", NULL);
    util::PlannerUtils::loadAdditionalRules(filePath, this->shootFromRules);
}
/**
 * @return String representation of goal predicate
 */
std::string GoalPlanner::determineGoal()
{

    // only consider safely reachable fields
    auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());
    std::unordered_set<std::shared_ptr<wumpus::model::Field>> possibleNextFields;
    //    if(localAgent->objective == wumpus::model::Objective::FETCH_OTHER_AGENT) {
    possibleNextFields = this->blockEval.generatePossibleNextFieldsAlternative();
    //    }

    std::string goalQueryHeadValue = "suggestedGoal(wildcard,wildcard)";
    auto result = this->extractor->extractReusableTemporaryQueryResult({goalQueryHeadValue}, "goal", this->goalGenerationRules);
    //    for(auto field : possibleNextFields) {
    //        field->updateIsPossibleNext(false);
    //    }
    this->integrator->applyChanges();
    if (!result.empty()) {
        if (result.size() > 1) {
            std::cerr << "PlanningModule: More than one suggested goal found. Using first entry. " << std::endl;
            throw std::exception();
        }
        auto ext = util::PlannerUtils::extractBinaryPredicateParameters(result.at(0), "suggestedGoal");
        std::stringstream goalRep;
        goalRep << "goal(" << ext.first << ',' << ext.second << ')';
        auto goal = goalRep.str();

        this->integrator->integrateInformationAsExternal(goal, "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        this->integrator->applyChanges();
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Determine goal success!" << std::endl;
#endif
        return goal;
    }
#ifdef PM_DEBUG
    std::cout << "PlanningModule: Determine goal not successful!" << std::endl;
#endif
    //    throw std::exception();
    return "";
}

std::pair<std::pair<std::string, std::string>, std::string> GoalPlanner::determinePosToShootFrom()
{
    auto position = std::pair<std::string, std::string>();
    auto heading = std::string();
    auto shotAt = std::vector<std::pair<std::string, std::string>>();
    auto result = this->extractor->extractReusableTemporaryQueryResult(
            {"targetPos(wildcard,wildcard)", "targetHeading(wildcard)", "fieldIsAhead(wildcard,wildcard)"}, "shootFrom", this->shootFromRules);

    // shooting wumpus should only be done from safe fields
    this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", false, aspkb::Strategy::INSERT_TRUE);
    this->integrator->applyChanges();
    if (!result.empty()) {

        for (const auto& res : result) {
#ifdef PM_DEBUG
            std::cout << "PlanningModule: Result for determinePosToShootFrom: " << std::endl;
            std::cout << res << ",";
#endif

            if (res.find("targetPos") != std::string::npos) {
                position = util::PlannerUtils::extractBinaryPredicateParameters(res, "targetPos");
            } else if (res.find("targetHeading") != std::string::npos) {
                heading = util::PlannerUtils::extractUnaryPredicateParameter(res);
            } else if (res.find("fieldIsAhead") != std::string::npos) {
                auto shotAtResult = util::PlannerUtils::extractBinaryPredicateParameters(res, "fieldIsAhead");
                shotAt.emplace_back(shotAtResult.first, shotAtResult.second);
            }
        }
#ifdef PM_DEBUG
        std::cout << std::endl;
#endif

        std::stringstream ss;
        ss << "goal(" << position.first << ", " << position.second << ")";
        this->integrator->integrateInformationAsExternal(ss.str(), "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        ss.str("");
        ss << "goalHeading(" << heading << ")";
        this->integrator->integrateInformationAsExternal(ss.str(), "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        ss.str("");

        if (this->wm->playground->getFieldsShotAtByAgentIds()->find(essentials::SystemConfig::getOwnRobotID()) !=
                this->wm->playground->getFieldsShotAtByAgentIds()->end()) {
            this->wm->playground->getFieldsShotAtByAgentIds()->at(essentials::SystemConfig::getOwnRobotID()).clear();
        }
        std::unordered_set<std::shared_ptr<wumpus::model::Field>> shotAtFields;
        for (const auto& field : shotAt) {
            this->wm->playground->addShootingTarget(essentials::SystemConfig::getOwnRobotID(), field);
            auto actualField = this->wm->playground->getField(std::stoi(field.first), std::stoi(field.second));
            shotAtFields.insert(actualField);
        }
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateShotAtFields(shotAtFields);

        this->integrator->applyChanges();
#ifdef PM_DEBUG
        std::cout << "Found Position to shoot from and integrated shooting targets" << std::endl;
#endif
        return std::make_pair(position, heading);
    } else {
#ifdef PM_DEBUG
        std::cout << "Could not determine position to shoot from " << std::endl;
// this is a special situation where a trap is blocking the agent to shoot at a useful position
#endif
        // TODO remove
        //        throw std::exception();
        return std::make_pair(std::make_pair("", ""), "");
    }
}
// void GoalPlanner::addShootingTarget(const int id, const std::pair<std::string, std::string>& shotAt)
//{
//    std::lock_guard<std::mutex> lock(this->shotAtMtx);
//    if (this->shootingTargets->find(id) != this->shootingTargets->end()) {
//        this->shootingTargets->at(id).insert(shotAt);
//    }
//    this->shootingTargets->emplace(id, std::set<std::pair<std::string, std::string>>({shotAt}));
//}
// std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> GoalPlanner::getShootingTargets()
//{
//    std::lock_guard<std::mutex> lock(this->shotAtMtx);
//    return this->shootingTargets;
//}
} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */
