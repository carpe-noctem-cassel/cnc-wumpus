#include "wumpus/wm/planning/PathActionsPlanner.h"
#include <aspkb/Integrator.h>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/wm/util/PlannerUtils.h>
#define PAP_DEBUG
namespace wumpus
{
namespace wm
{
namespace planning
{

PathActionsPlanner::PathActionsPlanner(aspkb::Extractor* extractor, aspkb::Integrator* integrator)
        : Planner(extractor)
        , integrator(integrator)
        , objEval(extractor, integrator)
        , goalPlanner(extractor, integrator)
        , blockEval(extractor, integrator)
        , exhaustionEval(extractor, integrator)
{
    auto sc = essentials::SystemConfig::getInstance();
    this->maxHorizonFactor = (*sc)[KB_CONFIG_NAME]->get<int>("maxHorizonFactor", NULL);
}
std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> PathActionsPlanner::tryDetermineActionsByObjective(
        const std::shared_ptr<wumpus::model::Agent>& agent)
{
    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> fieldsActionsPair;

    if ((this->wm->getEngine()->getAlicaClock()->now() - this->timePlanningStart).inSeconds() > this->wm->timeoutDurationSeconds) {
        std::vector<std::string> dummy = {"move", "turnLeft", "turnRight"};
        return std::make_pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>(
                std::vector<std::pair<std::string, std::string>>(), {dummy.at(rand() % dummy.size())});
    }

    if (this->objEval.agentObjectiveRequiresMovement(agent)) {

        if (this->objEval.agentObjectiveRequiresGoal(agent)) {
            auto goal = this->goalPlanner.determineGoal();
            if (goal.empty()) { // situation: agent wants to fetch other agent but cannot find a valid goal which means a wumpus or trap is blocking its way. if
                // it has an arrow, it should try to shoot
                // as many wumpi blocking both itself and others (might require new asp rules?) or if a trap is blocking it could go home already? TODO review
                if (this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->hasArrow &&
                        this->blockEval.determineWumpusBlocksSafeMovesForSelf()) {
                    this->goalPlanner.determinePosToShootFrom();
                } else { // FIXME hack
                    auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());
                    if (localAgent->currentPosition->x == localAgent->initialPosition->x && localAgent->currentPosition->y == localAgent->initialPosition->y) {
                        return std::make_pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>({}, {"leave"});
                    } else {
                        localAgent->updateObjective(model::Objective::GO_HOME);
                        this->integrator->applyChanges();
                        this->goalPlanner.determineGoal();
                    }
                }
            }
        } else if (agent->objective == model::Objective::HUNT_WUMPUS) {
            auto target = this->goalPlanner.determinePosToShootFrom();
            auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());
            if (!target.first.first.empty() && localAgent->currentPosition->x == std::stoi(target.first.first) &&
                    localAgent->currentPosition->y == std::stoi(target.first.second) && localAgent->currentHeading == std::stoi(target.second)) {
                std::cout << "PathActionsPlanner: TARGET " << target.first.first << ", " << target.first.second << " with heading " << target.second
                          << std::endl;
                return std::make_pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>({}, {"shoot"});
            }
        }

        //        this->determineAllAgentsExhausted(); // FIXME experimental
        fieldsActionsPair = tryGetSafeActions(agent);

        if (fieldsActionsPair.second.empty()) {
#ifdef PAP_DEBUG
            std::cout << "PathActionsPlanner: exhausted options 1!" << std::endl;
#endif
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
            this->integrator->applyChanges();
            this->objEval.determineObjective();
            auto possiblyNewActions = tryDetermineActionsByObjective(agent); // tryGetSafeActions(agent);// <- objective might have changed here!
            if (!possiblyNewActions.second.empty()) {
                return possiblyNewActions;
            }
            return std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>();
        }

    } else if (this->objEval.objectiveImpliesSimpleAction(agent)) {
#ifdef PAP_DEBUG
        std::cout << "PathActionsPlanner: Objective implies simple action" << std::endl;
#endif
        fieldsActionsPair = this->determineActionsNoMovement();

    } else if (agent->objective == wumpus::model::Objective::IDLE) {
#ifdef PAP_DEBUG
        std::cout << "PathActionsPlanner: Objective implies idling!" << std::endl;
#endif
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
        fieldsActionsPair = std::make_pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>(
                {}, std::vector<std::string>({(random() % 2 == 0 ? "turnLeft" : "turnRight")}));
    }
    return fieldsActionsPair;
}

std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> PathActionsPlanner::tryGetSafeActions(
        const std::shared_ptr<wumpus::model::Agent>& agent)
{
    auto fieldsActionsPair =
            determinePathAndActions(agent->objective == wumpus::model::Objective::GO_HOME, agent->objective == wumpus::model::Objective::MOVE_TO_GOLD_FIELD);

    // Incremental approach couldn't find a solution - try to shoot wumpus or allow unsafe moves
    if (fieldsActionsPair.second.empty()) {
#ifdef PAP_DEBUG
        std::cout << "PathActionsPlanner: Could not determine Path and Actions" << std::endl;
#endif
        if (this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective == wumpus::model::Objective::IDLE) {
            std::cout << "Agent should idle but is determining blockage status instead! " << std::endl;
            throw std::exception();
        }

        auto blockStatus = blockEval.determineBlockingElementsForSelf();
        if (!blockStatus.allPathsBlocked) {
            std::cout << "Could not determine actions but blockage evaluator indicates safe moves should still be possible! (might be when trying to move to a "
                         "goal) "
                      << std::endl; // TODO move goal to a safe field which, if explored might rule out danger, if possible?
            if (this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective == wumpus::model::Objective::EXPLORE ||
                    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective == wumpus::model::Objective::MOVE_TO_GOLD_FIELD) {
                this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", true, aspkb::Strategy::INSERT_TRUE);
                this->integrator->applyChanges();
            }

            //            throw std::exception();
        }

        if (!blockStatus.blockingWumpi.empty()) {
#ifdef PAP_DEBUG
            std::cout << "PathActionsPlanner: Wumpus blocks safe moves!" << std::endl;
#endif
            if (wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->hasArrow) {
#ifdef PAP_DEBUG
                std::cout << "PathActionsPlanner: Agent has arrow! Trying to shoot wumpus" << std::endl;
#endif
                this->objEval.determineObjective();
                std::pair<std::pair<std::string, std::string>, std::string> possibleShootingPosition;
                if (agent->objective == wumpus::model::Objective::HUNT_WUMPUS) {

                    possibleShootingPosition = this->goalPlanner.determinePosToShootFrom();
                }

                if (agent->objective == wumpus::model::Objective::HUNT_WUMPUS && !possibleShootingPosition.second.empty()) { // might idle instead
                    fieldsActionsPair = determinePathAndActions(false, false, true);
//                    fieldsActionsPair = tryDetermineActionsByObjective(agent);
#ifdef PAP_DEBUG
                    if (fieldsActionsPair.second.empty()) {
                        std::cout << "PathActionsPlanner: Could not determine Path and Actions to shooting position but this should never fail as only visited "
                                     "fiedls are considered as possible goals "
                                  << fieldsActionsPair.second.size() << std::endl;
                        //                        throw std::exception();
                    }

#endif
                } else { // TODO correct? cant do anything about wumpus -> set exhausted and try again
#ifdef PAP_DEBUG
                    std::cout << "PathActionsPlanner: Objective is not hunt wumpus or no position to shoot from found. Setting exhausted to true and allowing "
                                 "unsafe moves."
                              << std::endl;
#endif
                    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
                    // FIXME hack
                    if (agent->objective == wumpus::model::Objective::HUNT_WUMPUS) {
                        this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", true, aspkb::Strategy::INSERT_TRUE);
                    } else {
                        std::cout << "integrator is at " << this->integrator << std::endl;
                        this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves",
                                this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->exhausted, aspkb::Strategy::INSERT_TRUE);
                    }

                    this->integrator->applyChanges();
#ifdef PAP_DEBUG
                    std::cout << "PathActionsPlanner: Determining objective after setting unsafeMoves and try to find actions by objective!" << std::endl;
#endif
                    this->objEval.determineObjective();
                    fieldsActionsPair = tryDetermineActionsByObjective(agent);
                }
            } else {
#ifdef PAP_DEBUG
                std::cout << "PathActionsPlanner: WumpusBlocksMoves but agent doesn't have arrow!" << std::endl;

#endif
                this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
                this->integrator->applyChanges();
                this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves",
                        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->exhausted, aspkb::Strategy::INSERT_TRUE);
                this->integrator->applyChanges();
#ifdef PAP_DEBUG
                std::cout << "PathActionsPlanner: Determining objective after setting unsafeMoves and try to find actions by objective!" << std::endl;
#endif
                this->objEval.determineObjective();
                fieldsActionsPair = tryDetermineActionsByObjective(agent);
            }
        } else if (!blockStatus.blockingTraps.empty()) { // no blocking wumpi found - trap must be blocking (TODO confirm).
#ifdef PAP_DEBUG
            std::cout << "PathActionsPlanner: Trap blocks self - setting exhausted and possibly unsafeMovesAllowed!" << std::endl;
#endif
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
            this->integrator->applyChanges();

            // FIXME experimental - 24.06.20 - unsafe moves should only be allowed if all other agents are blocked by traps as well

            bool foundUntrappedAgent = false;
            for (const auto& entry : *this->wm->playground->getAgents(false)) {
                if (!entry.second->isBlockedByTrap() && this->wm->wumpusSimData.communicationAllowed) {
                    foundUntrappedAgent = true;
                    std::cout << "Found untrapped agent with id " << entry.second->id << "!" << std::endl;
                    break;
                }
            }

            if (!foundUntrappedAgent) {
                std::cout << "Couldn't find an untrapped agent!" << std::endl;
            }

//            this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", true, aspkb::Strategy::INSERT_TRUE);
            this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", !foundUntrappedAgent, aspkb::Strategy::INSERT_TRUE);
            //            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(false);
            this->integrator->applyChanges();

#ifdef PAP_DEBUG
            std::cout << "PathActionsPlanner: determine Objective and try getting actions by objective after setting exhausted self" << std::endl;
#endif
            this->objEval.determineObjective();
            fieldsActionsPair = tryDetermineActionsByObjective(agent); // tryGetSafeActions(agent);
        } else {
            std::cout << "All safe paths are blocked but no problematic wumpi or traps have been found!" << std::endl;
            //            throw std::exception(); // FIXME evaluate - this was only other branch before
        }
    }
    auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());

    // agent moved out of a blocking situation and should consider itself unblocked
    //    if (std::find(fieldsActionsPair.second.begin(), fieldsActionsPair.second.end(), "move") != fieldsActionsPair.second.end() &&
    //            (localAgent->isBlockedByWumpus() || localAgent->isBlockedByWumpus())) {
    //        localAgent->updateBlockingWumpi(std::unordered_set<std::shared_ptr<wumpus::model::Field>>());
    //        localAgent->updateBlockingTraps(std::unordered_set<std::shared_ptr<wumpus::model::Field>>());
    //        this->integrator->applyChanges();
    //    }

    return fieldsActionsPair;
}

std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> PathActionsPlanner::determinePathAndActions(
        bool isGoingHome, bool isGoingToGold, bool isHuntingWumpus)
{
    std::string pathPredicate = "on";
    std::string actionsPredicate = "occurs";
    //    std::string movedInDangerPredicate = "movedInDanger"

    // TODO think of something more suitable
    auto startHorizon = 1;
    auto maxHorizon = this->maxHorizonFactor * this->wm->playground->getPlaygroundSize();

    std::string pathQueryValue = "holds(on(wildcard,wildcard), wildcard)";
    std::string actionsQueryValue = "occurs(wildcard,wildcard)";

    if (!this->pathAndActionsProblem) {

        this->pathAndActionsProblem = std::make_shared<aspkb::IncrementalProblem>(aspkb::TermManager::getInstance().getSolver(),
                std::vector<std::string>({pathQueryValue, actionsQueryValue}), std::map<std::string, std::string>(), KB_CONFIG_NAME,
                "pathAndActionsIncrementalProblem", "pathActions", startHorizon, maxHorizon, false);
    }

//    stepTerm->addRule("{occurs(A,t-1) : moveAction(A)} = 1."); //FIXME was in Extractor before (problems while loading from disk???)
#ifdef PAP_DEBUG
    std::cout << "PathActionsPlanner: Determining actions with incquery: Check if all agents are exhausted" << std::endl;
//    this->determineAllAgentsExhausted();
#endif

    int overrideHorizon;
    if (isGoingHome || isGoingToGold) {
        overrideHorizon = 4 * this->wm->playground->getPlaygroundSize();
    } else if (isHuntingWumpus) {
        overrideHorizon = 2 * this->wm->playground->getPlaygroundSize();
    } else {
        overrideHorizon = -1;
    }

    auto result = this->extractor->solveWithIncrementalExtensionQuery(this->pathAndActionsProblem, overrideHorizon);

    auto path = std::vector<std::pair<std::string, std::string>>();
    auto actions = std::vector<std::string>();

    if (!result.empty()) {
#ifdef PAP_DEBUG
        std::cout << "PathActionsPlanner: Found actions " << std::endl;
#endif
        for (const auto& elem : result) {
            auto pathResult = util::PlannerUtils::extractBinaryPredicateParameters(elem, "on");
            if (!pathResult.first.empty() && !pathResult.second.empty()) {
                path.emplace_back(pathResult.first, pathResult.second);
            }
            auto actionsResult = util::PlannerUtils::extractBinaryPredicateParameters(elem, "occurs");
            if (!actionsResult.first.empty() && !actionsResult.second.empty()) {
                if (std::stoi(actionsResult.second) + 1 > actions.size()) {
                    actions.resize(std::stoi(actionsResult.second) + 1);
                }
#ifdef PAP_DEBUG
                std::cout << "PathActionsPlanner: adding action" << actionsResult.first << std::endl;
#endif
                actions[std::stoi(actionsResult.second)] = actionsResult.first;
            }
        }
    } else {
#ifdef PAP_DEBUG
        std::cout << "PathActionsPlanner: Couldn't determine actions with incquery " << std::endl;
#endif
    }

    return std::make_pair(path, actions);
}

/**
 * Activates a query with simple rules for getting the next action when no movement is required
 * @return
 */
std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> PathActionsPlanner::determineActionsNoMovement()
{
    auto result = std::vector<std::string>();
    auto inquiry = "occurs(wildcard)";
    std::vector<std::string> rules;
    auto rule = "occurs(pickup) :- on(X,Y), glitter(X,Y), not haveGold(A), me(A)";
    rules.emplace_back(rule);
    rule = "occurs(shoot) :- on(X,Y), goal(X,Y), heading(H), goalHeading(H)";
    rules.emplace_back(rule);
    rule = "occurs(leave) :- on(X,Y), haveGold(A), me(A), initial(X,Y), not occurs(shoot)";
    rules.emplace_back(rule);

    result = this->extractor->extractReusableTemporaryQueryResult({inquiry}, "simpleAction", rules);
    return std::make_pair<std::vector<std::pair<std::string, std::string>>>({}, result);
}
} /* namespace planning */
} /* namespace wm */
} /* namespace wumpus */