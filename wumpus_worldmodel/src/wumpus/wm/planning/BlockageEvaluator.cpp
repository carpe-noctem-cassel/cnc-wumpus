#include "wumpus/wm/planning/BlockageEvaluator.h"
#include <aspkb/Integrator.h>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/wm/util/PlannerUtils.h>
#define BE_DEBUG
namespace wumpus
{
namespace wm
{
namespace planning
{

static std::vector<std::string> blockedByAnyRules = {
        "wumpusPossibleOrDefinite(X,Y) :- wumpusPossible(X,Y).", "wumpusPossibleOrDefinite(X,Y) :- wumpus(X,Y).",
        "trapPossibleOrDefinite(X,Y) :- trapPossible(X,Y).", "trapPossibleOrDefinite(X,Y) :- trap(X,Y).", "exploredOrVisited(X,Y) :- explored(X,Y).",
        "exploredOrVisited(X,Y) :- visited(X,Y).", "dangerPossibleOrDefinite(X,Y) :- wumpusPossibleOrDefinite(X,Y).",
        "dangerPossibleOrDefinite(X,Y) :- trapPossibleOrDefinite(X,Y).",

        "blockWumpus(X,Y) :- wumpusPossibleOrDefinite(X,Y), possibleNext(X,Y), allPathsBlocked.",
        "blockTrap(X,Y) :- trapPossibleOrDefinite(X,Y), possibleNext(X,Y), allPathsBlocked.",

        "notAllPathsBlocked :- possibleNext(X,Y), not dangerPossibleOrDefinite(X,Y).",

        "notAllPathsBlocked :- objective(A,shoot), me(A).", // rules only make sense in certain objectives. these have to be considered here because
        // the objective depends on the result of this operation
        "notAllPathsBlocked :- objective(A,collectGold), me(A).",
        "notAllPathsBlocked :- objective(A,idle), me(A).", // FIXME idle here correct?
        "notAllPathsBlocked :- not possibleNext(_,_)", "notAllPathsBlocked :- objective(A,leave), me(A).",
        "allPathsBlocked :- field(_,_), not notAllPathsBlocked." // field(_,_) is a hack because parsing fails for some reason TODO investigate
                                                                 // why, probably expandRuleMP
};

static std::vector<std::string> blockedByWumpusRules = {
        "wumpusPossibleOrDefinite(X,Y) :- wumpusPossible(X,Y).", "wumpusPossibleOrDefinite(X,Y) :- wumpus(X,Y).",
        "trapPossibleOrDefinite(X,Y) :- trapPossible(X,Y).", "trapPossibleOrDefinite(X,Y) :- trap(X,Y).", "exploredOrVisited(X,Y) :- explored(X,Y).",
        "exploredOrVisited(X,Y) :- visited(X,Y).", "blockWumpus(X,Y) :- wumpusPossibleOrDefinite(X,Y), possibleNext(X,Y).",

        "notAllPathsBlockedByWumpus :- possibleNext(X,Y), not blockWumpus(X,Y), not objective(A, fetchOtherAgent), me(A), not trapPossible(X,Y), not "
        "trap(X,Y).",
        "notAllPathsBlockedByWumpus :- me(A), objective(A,fetchOtherAgent), not blockWumpus(X,Y) : possibleNext(X,Y).", // FIXME review

        "notAllPathsBlocked :- field(_,_), notAllPathsBlockedByWumpus.",

        "notAllPathsBlocked :- objective(A,shoot), me(A).", // rules only make sense in certain objectives. these have to be considered here because
        // the objective depends on the result of this operation
        "notAllPathsBlocked :- objective(A,collectGold), me(A).",
        "notAllPathsBlocked :- objective(A,idle), me(A).", // FIXME idle here correct?
        "notAllPathsBlocked :- not possibleNext(_,_)", "notAllPathsBlocked :- objective(A,leave), me(A).",
        "allPathsBlocked :- field(_,_), not notAllPathsBlocked." // field(_,_) is a hack because parsing fails for some reason TODO investigate
                                                                 // why, probably expandRuleMP
};

static std::vector<std::string> blockedByTrapRules = {
        "wumpusPossibleOrDefinite(X,Y) :- wumpusPossible(X,Y).", "wumpusPossibleOrDefinite(X,Y) :- wumpus(X,Y).",
        "trapPossibleOrDefinite(X,Y) :- trapPossible(X,Y).", "trapPossibleOrDefinite(X,Y) :- trap(X,Y).", "exploredOrVisited(X,Y) :- explored(X,Y).",
        "exploredOrVisited(X,Y) :- visited(X,Y).", "blockTrap(X,Y) :- trapPossibleOrDefinite(X,Y), possibleNext(X,Y).",

        "notAllPathsBlockedByTrap :- possibleNext(X,Y), not blockTrap(X,Y), not objective(A, fetchOtherAgent), me(A).",
        "notAllPathsBlockedByTrap :- me(A), objective(A,fetchOtherAgent), not blockTrap(X,Y) : possibleNext(X,Y).", // FIXME review

        "notAllPathsBlocked :- field(_,_), notAllPathsBlockedByTrap.",

        "notAllPathsBlocked :- objective(A,shoot), me(A).", // rules only make sense in certain objectives. these have to be considered here because
        // the objective depends on the result of this operation
        "notAllPathsBlocked :- objective(A,collectGold), me(A).",
        //        "notAllPathsBlocked :- objective(A,idle), me(A).", // FIXME idle here correct?
        "notAllPathsBlocked :- not possibleNext(_,_)", "notAllPathsBlocked :- objective(A,leave), me(A).",
        "allPathsBlocked :- field(_,_), not notAllPathsBlocked." // field(_,_) is a hack because parsing fails for some reason TODO investigate
                                                                 // why, probably expandRuleMP

};
BlockageEvaluator::BlockageEvaluator(aspkb::Extractor* extractor, aspkb::Integrator* integrator)
        : Planner(extractor)
        , integrator(integrator)
{
}
/**
 * the feedback of this has been moved into the knowledge base
 * @return
 */
bool BlockageEvaluator::determineWumpusBlocksSafeMovesForSelf()
{
    this->generatePossibleNextFields();

    auto ret = this->extractor->extractReusableTemporaryQueryResult(
            {"allPathsBlocked", "blockWumpus(wildcard,wildcard)"}, "blockingWumpusExists", blockedByWumpusRules);

    // FIXME something very wrong here. results for one query contained inquiry predicates from another... HACK
    auto possiblyBlockingWumpi = std::unordered_set<std::shared_ptr<wumpus::model::Field>>();
    bool found = false;
    for (const auto& r : ret) {
#ifdef BE_DEBUG
        std::cout << "BlockageEvaluator: R IN RET FOR DETERMINEWUMPUSBLOCKSSAFEMOVES: " << r << std::endl;
#endif
        if (r == "allPathsBlocked") {
            found = true;
        } else if (r.find("blockWumpus") != std::string::npos) {
            auto coords = util::PlannerUtils::extractBinaryPredicateParameters(r, "blockWumpus");
            possiblyBlockingWumpi.insert(this->wm->playground->getField(std::stoi(coords.first), std::stoi(coords.second)));
        }
    }
#ifdef BE_DEBUG
    std::cout << "BlockageEvaluator: blocking wumpus exists?: " << (found ? "True" : "False") << std::endl;
#endif

    if (found) {
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateBlockingWumpi(possiblyBlockingWumpi);
    }
    if (!found && this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective == wumpus::model::Objective::HUNT_WUMPUS) {
        this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        this->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    }
    this->integrator->applyChanges();
    //    throw std::exception();
    return found;
}
std::unordered_set<std::shared_ptr<wumpus::model::Field>> BlockageEvaluator::generatePossibleNextFields()
{
    auto playground = this->wm->playground;

    auto visitedFields = std::vector<std::shared_ptr<model::Field>>();
    for (int i = 0; i < playground->getPlaygroundSize(); ++i) {
        for (int j = 0; j < playground->getPlaygroundSize(); ++j) {
            auto field = playground->getField(i, j);
            if (field->visited) {
                visitedFields.emplace_back(field);
            }
        }
    }

    std::set<std::string> possibleNextFields;
    for (auto visited : visitedFields) {
        std::shared_ptr<aspkb::PossibleNextFieldGenerationProblem> problem = nullptr;
        auto coordinatePair = std::pair<int, int>(visited->x, visited->y);
        if (possibleNextFromProblems.find(coordinatePair) != possibleNextFromProblems.end()) {
            problem = possibleNextFromProblems.at(coordinatePair);
        } else {
            auto x = std::to_string(visited->x);
            auto y = std::to_string(visited->y);
            auto externalPrefix = std::string("possibleNext").append(x).append(y);
            std::map<std::string, std::string> baseTermParameters = {{"x", x}, {"y", y}};
            problem = std::make_shared<aspkb::PossibleNextFieldGenerationProblem>(
                    aspkb::TermManager::getInstance().getSolver(), baseTermParameters, externalPrefix, 2 * wm->playground->getPlaygroundSize());
            possibleNextFromProblems.emplace(coordinatePair, problem);
        }
        auto possibleNextFieldsForIteration = problem->doIncrementalSolving();
        for (auto field : possibleNextFieldsForIteration) {
            possibleNextFields.insert(field);
        }
    }

    std::unordered_set<std::shared_ptr<wumpus::model::Field>> actualFields;
    for (auto elem : possibleNextFields) {
        auto params = util::PlannerUtils::extractBinaryPredicateParameters(elem, "possibleNextCandidate");
        auto affectedField = playground->getField(std::stoi(params.first), std::stoi(params.second));
        affectedField->updateIsPossibleNext(true);
        integrator->applyChanges();
        actualFields.insert(affectedField);
    }
    return actualFields;
}

bool BlockageEvaluator::determineTrapBlocksSafeMovesForSelf()
{

    this->generatePossibleNextFields();

    auto ret =
            this->extractor->extractReusableTemporaryQueryResult({"allPathsBlocked", "blockTrap(wildcard,wildcard)"}, "blockingTrapExists", blockedByTrapRules);

    // FIXME something very wrong here. results for one query contained inquiry predicates from another... HACK
    auto possiblyBlockingTraps = std::unordered_set<std::shared_ptr<wumpus::model::Field>>();
    bool found = false;
    for (const auto& r : ret) {
#ifdef BE_DEBUG
        std::cout << "BlockageEvaluator: R IN RET FOR determineTrapBlocksSafeMoves: " << r << std::endl;
#endif
        if (r == "allPathsBlocked") {
            found = true;
        } else if (r.find("blockTrap") != std::string::npos) {
            auto coords = util::PlannerUtils::extractBinaryPredicateParameters(r, "blockTrap");
            possiblyBlockingTraps.insert(this->wm->playground->getField(std::stoi(coords.first), std::stoi(coords.second)));
        }
    }
#ifdef BE_DEBUG
    std::cout << "BlockageEvaluator: blocking trap exists?: " << (found ? "True" : "False") << std::endl;
#endif

    if (found) {
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateBlockingTraps(possiblyBlockingTraps);
    }
    if (!found && this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective == wumpus::model::Objective::HUNT_WUMPUS) {
        this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        this->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    }
    this->integrator->applyChanges();
    //    throw std::exception();
    return found;
}

BlockageStatus BlockageEvaluator::determineBlockingElementsForSelf()
{
    auto possibleNextFields = this->generatePossibleNextFields();

    auto ret = this->extractor->extractReusableTemporaryQueryResult(
            {"allPathsBlocked", "blockWumpus(wildcard,wildcard)", "blockTrap(wildcard,wildcard)"}, "blockingElements", blockedByAnyRules);

    auto possiblyBlockingWumpi = std::unordered_set<std::shared_ptr<wumpus::model::Field>>();
    auto possiblyBlockingTraps = std::unordered_set<std::shared_ptr<wumpus::model::Field>>();
    bool found = false;
    for (const auto& r : ret) {
#ifdef BE_DEBUG
        std::cout << "BlockageEvaluator: R IN RET FOR determineBlockingElementsForSelf: " << r << std::endl;
#endif
        if (r == "allPathsBlocked") {
            found = true;
        } else if (r.find("blockWumpus") != std::string::npos) {
            auto coords = util::PlannerUtils::extractBinaryPredicateParameters(r, "blockWumpus");
            possiblyBlockingWumpi.insert(this->wm->playground->getField(std::stoi(coords.first), std::stoi(coords.second)));
        } else if (r.find("blockTrap") != std::string::npos) {
            auto coords = util::PlannerUtils::extractBinaryPredicateParameters(r, "blockTrap");
            possiblyBlockingTraps.insert(this->wm->playground->getField(std::stoi(coords.first), std::stoi(coords.second)));
        }
    }
#ifdef BE_DEBUG
    std::cout << "BlockageEvaluator: blocking any exists?: " << (found ? "True" : "False") << std::endl;
#endif

    //    if (found) {
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateBlockingWumpi(possiblyBlockingWumpi);
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateBlockingTraps(possiblyBlockingTraps);
    //    }
    if (!found && this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective == wumpus::model::Objective::HUNT_WUMPUS) {
        this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        this->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    }
    this->integrator->applyChanges();

    // reset possible Next fields
    for (const auto& field : possibleNextFields) {
        field->updateIsPossibleNext(false);
    }
    return wumpus::wm::planning::BlockageStatus(possiblyBlockingWumpi, possiblyBlockingTraps, found);
}
}
}
}
