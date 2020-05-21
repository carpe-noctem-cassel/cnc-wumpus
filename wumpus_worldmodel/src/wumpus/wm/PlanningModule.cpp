#include "wumpus/wm/PlanningModule.h"
#include "wumpus/WumpusWorldModel.h"
#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <aspkb/Extractor.h>
#include <aspkb/IncrementalProblem.h>
#include <aspkb/Integrator.h>
#include <aspkb/PathExistsProblem.h>
#include <aspkb/Strategy.h>
#include <chrono>
#include <engine/AlicaEngine.h>
#include <iostream>
#include <memory>
#include <random>
#include <reasoner/asp/Solver.h>
#include <sstream>
#include <thread>
#include <wumpus/model/Agent.h>
#include <wumpus/model/Field.h>

#define PM_DEBUG
namespace wumpus
{
namespace wm
{

PlanningModule::PlanningModule(wumpus::WumpusWorldModel* wm)
        : wm(wm)
        , shootingTargets(std::make_shared<std::map<int, std::set<std::pair<std::string, std::string>>>>())
        , pathAndActionsProblem(nullptr)
{
    this->extractor = new aspkb::Extractor();

    // TODO reduce code
    this->sc = essentials::SystemConfig::getInstance();
    auto filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("goalGenerationRulesFilePath", NULL);
    this->loadAdditionalRules(filePath, this->goalGenerationRules);
    filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("actionsGenerationRulesFilePath", NULL);
    this->loadAdditionalRules(filePath, this->actionsGenerationRules);
    filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("actionsGenWumpusFilePath", NULL);
    this->loadAdditionalRules(filePath, this->actionsGenWumpusRules);
    filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("pathValidationRulesFilePath", NULL);
    this->loadAdditionalRules(filePath, this->pathValidationRules);
    filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("objectiveRulesFilePath", NULL);
    this->loadAdditionalRules(filePath, this->objectiveRules);
    filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("shootFromRulesFilePath", NULL);
    this->loadAdditionalRules(filePath, this->shootFromRules);
    filePath = (*sc)[KB_CONFIG_NAME]->get<std::string>("allAgentsExhaustedFilePath", NULL);
    this->loadAdditionalRules(filePath, this->allAgentsExhaustedRules);

    this->maxHorizonFactor = (*sc)[KB_CONFIG_NAME]->get<int>("maxHorizonFactor", NULL);

    this->isPlanning = false;
}

PlanningModule::~PlanningModule()
{
    delete this->extractor;
}

/**
 *
 * @param agent
 * @return
 */
std::pair<int, std::vector<WumpusEnums::actions>> PlanningModule::processNextActionRequest(std::shared_ptr<wumpus::model::Agent> agent)
{

    this->wm->integrateChanges();
    this->wm->wumpusSimData.setIntegratedFromSimulator(false);
    this->setIsPlanning(true);

    auto actions = std::vector<WumpusEnums::actions>();
    auto params = std::vector<std::string>();
    std::stringstream ss;

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> fieldsActionsPair;
    std::string goal;

    // FIXME does this make sense here?

    //    if(!this->wm->playground->goldFieldKnown) {
    if (this->wm->playground->wumpusBlocksSafeMoves && !this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->hasGold) { //TODO gold hack as this breaks hunting wumpi for other agents
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Checking if Wumpus still blocks safe moves!" << std::endl;
#endif
        this->determineWumpusBlocksSafeMoves();
    }
    //    }

    // determine objective - might cause replan necessary to be set
    this->determineAllAgentsExhausted(); // experimental TODO

    auto obj = this->determineObjective();
    // determine actions
    if (agent->replanNecessary || this->lastPathAndActions.second.empty()) { // FIXME this seems to be problematic. make sure replan necessary is set
        //    correctly for all occasions
        do {
            std::cout << "~~~~~~~~~~~~~~~~~~~ PLANNING " << std::endl;
            fieldsActionsPair = this->tryDetermineActionsByObjective(agent);

        } while (fieldsActionsPair.second.empty());
    } else { // no replan necessary // lastPathAndActions not empty

        fieldsActionsPair = this->lastPathAndActions;
    }

    this->lastPathAndActions = fieldsActionsPair;
    auto result = fieldsActionsPair.second;
    WumpusEnums::actions actionArray[result.size()];
    for (int i = 0; i < result.size(); ++i) {
        auto tmp = result.at(i);
        auto index = i;

        if (tmp.find("move") != std::string::npos) {
            actionArray[index] = WumpusEnums::move;
        } else if (tmp.find("turnRight") != std::string::npos) {
            actionArray[index] = WumpusEnums::turnRight;
        } else if (tmp.find("turnLeft") != std::string::npos) {
            actionArray[index] = WumpusEnums::turnLeft;
        } else if (tmp.find("shoot") != std::string::npos) {
            //            std::cout << agent->currentPosition->x << ", " << agent->currentPosition->y << std::endl;
            actionArray[index] = WumpusEnums::shoot;
            // reset goal and goalHeading
            // reset wumpus blocks safe moves TODO right place?
            this->wm->playground->updateWumpusBlocksMoves(false);
            this->wm->changeHandler->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
            this->wm->changeHandler->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
            std::cout << "SHOT!" << std::endl;
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateShot();
            this->wm->integrateChanges();
        } else if (tmp.find("pickup") != std::string::npos) {
            actionArray[index] = WumpusEnums::pickUpGold;
        } else if (tmp.find("leave") != std::string::npos) {
            actionArray[index] = WumpusEnums::leave;
        } else {
            std::cout << "KM: UNKNOWN ACTION! " << tmp << std::endl;
            // TODO REMOVE
            throw std::exception();
            actionArray[0] = WumpusEnums::move;
        }
    }
    for (int i = 0; i < result.size(); ++i) {
        actions.push_back(actionArray[i]);
    }

    agent->replanNecessary = false;
    this->setIsPlanning(false);
    //        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (this->lastPathAndActions.second.size() > 0) {
        // minus first action (should have been performed)
        this->lastPathAndActions.second.erase(this->lastPathAndActions.second.begin());
    }

    // fixme
    return std::make_pair(0, actions);
}

std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> PlanningModule::tryGetSafeActions(
        const std::shared_ptr<wumpus::model::Agent>& agent)
{
    auto fieldsActionsPair = determinePathAndActions();

    // Incremental approach couldn't find a solution - try to shoot wumpus or allow unsafe moves
    if (fieldsActionsPair.second.empty()) {
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Could not determine Path and Actions" << std::endl;
#endif
        if (determineWumpusBlocksSafeMoves()) {
#ifdef PM_DEBUG
            std::cout << "PlanningModule: Wumpus blocks safe moves!" << std::endl;
#endif
            if (wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->hasArrow) {
#ifdef PM_DEBUG
                std::cout << "PlanningModule: Agent has arrow! Trying to shoot wumpus" << std::endl;
#endif
                this->determineObjective();
                if (agent->objective == wumpus::model::Objective::HUNT_WUMPUS && !this->determinePosToShootFrom().second.empty()) { // might idle instead
                    fieldsActionsPair = determinePathAndActions();
//                    fieldsActionsPair = tryDetermineActionsByObjective(agent);
#ifdef PM_DEBUG
                    if (fieldsActionsPair.second.empty()) {
                        std::cout << "PlanningModule: Could not determine Path and Actions to shooting position but this should never fail as only visited "
                                     "fiedls are considered as possible goals "
                                  << fieldsActionsPair.second.size() << std::endl;
                        throw std::exception();
                    }

#endif
                } else { // TODO correct? cant do anything about wumpus -> set exhausted and try again
#ifdef PM_DEBUG
                    std::cout << "PlanningModule: Objective is not hunt wumpus or no position to shoot from found. Setting exhausted to true and allowing "
                                 "unsafe moves."
                              << std::endl;
#endif
                    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
                    this->wm->changeHandler->integrator->integrateInformationAsExternal(
                            "unsafeMovesAllowed", "unsafeMoves", this->determineAllAgentsExhausted(), aspkb::Strategy::INSERT_TRUE);
                    this->wm->integrateChanges();
#ifdef PM_DEBUG
                    std::cout << "PlanningModule: Determining objective after setting unsafeMoves and try to find actions by objective!" << std::endl;
#endif
                    this->determineObjective();
                    fieldsActionsPair = tryDetermineActionsByObjective(agent);
                }
            } else {
#ifdef PM_DEBUG
                std::cout << "PlanningModule: WumpusBlocksMoves but agent doesn't have arrow!" << std::endl;

#endif
                this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
                this->wm->integrateChanges();
                this->wm->changeHandler->integrator->integrateInformationAsExternal(
                        "unsafeMovesAllowed", "unsafeMoves", this->determineAllAgentsExhausted(), aspkb::Strategy::INSERT_TRUE);
                this->wm->integrateChanges();
#ifdef PM_DEBUG
                std::cout << "PlanningModule: Determining objective after setting unsafeMoves and try to find actions by objective!" << std::endl;
#endif
                this->determineObjective();
                fieldsActionsPair = tryDetermineActionsByObjective(agent);
            }
        } else { // no blocking wumpi found - trap must be blocking (TODO confirm).
#ifdef PM_DEBUG
            std::cout << "PlanningModule: No blocking wumpi found - setting exhausted and possibly unsafeMovesAllowed!" << std::endl;
#endif
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
            this->wm->integrateChanges();

            // FIXME experimental
            this->wm->changeHandler->integrator->integrateInformationAsExternal(
                    "unsafeMovesAllowed", "unsafeMoves", this->determineAllAgentsExhausted(), aspkb::Strategy::INSERT_TRUE);
            this->wm->integrateChanges();
#ifdef PM_DEBUG
            std::cout << "PlanningModule: determine Objective and try getting actions by objective after setting exhausted self" << std::endl;
#endif
            this->determineObjective();
            fieldsActionsPair = tryDetermineActionsByObjective(agent); // tryGetSafeActions(agent);
        }
    }
    return fieldsActionsPair;
}

/**
 * @return String representation of goal predicate
 */
std::string PlanningModule::determineGoal()
{
    std::string goalQueryHeadValue = "suggestedGoal(wildcard,wildcard)";

    auto result = this->extractor->extractReusableTemporaryQueryResult({goalQueryHeadValue}, "goal", this->goalGenerationRules);
    if (!result.empty()) {
        if (result.size() > 1) {
            std::cerr << "PlanningModule: More than one suggested goal found. Using first entry. " << std::endl;
            throw std::exception();
        }
        auto ext = this->extractBinaryPredicateParameters(result.at(0), "suggestedGoal");
        std::stringstream goalRep;
        goalRep << "goal(" << ext.first << ',' << ext.second << ')';
        auto goal = goalRep.str();

        this->wm->changeHandler->integrator->integrateInformationAsExternal(goal, "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        this->wm->integrateChanges();
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

// TODO
/**
 * Extracts the current objective and registers it as an external along with agent Id
 * @return
 */
wumpus::model::Objective PlanningModule::determineObjective()
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
    if (this->wm->playground->getAgentById(essentials::SystemConfig::getInstance()->getOwnRobotID())->hasGold) {
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Own Agent has gold - checking if others need help" << std::endl;
#endif
        auto positionsOfOtherAgents = std::map<int, std::pair<int, int>>();
        auto goldField = std::pair<int, int>();
        auto allAgents = this->wm->playground->getAgents(false);
        for (const auto& agent : *allAgents) {
            if (agent.first != essentials::SystemConfig::getInstance()->getOwnRobotID() && agent.second && agent.second->currentPosition &&
                    !agent.second->hasSafePathToGold) {
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
        this->checkSafePathsExistsForOtherAgents(goldField, positionsOfOtherAgents);
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
        this->wm->integrateChanges();
        //        wumpus::model::Objective obj = wumpus::model::Objective::UNDEFINED;
        //        obj = this->determineObjective();
        auto sol2 = this->extractor->extractReusableTemporaryQueryResult({"objective(wildcard)"}, "objective", this->objectiveRules);
        if (sol2.empty()) {
            std::cout << "PlanningModule: Should have a posible objective by now!" << std::endl;
            throw std::exception();
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
        if (this->wm->playground->getAgentById(essentials::SystemConfig::getInstance()->getOwnRobotID())->objective != obj) {
#ifdef PM_DEBUG
            std::cout << "PlanningModule: objective changed -> setting unsafe moves allowed to false" << std::endl;
#endif
            this->wm->changeHandler->integrator->integrateInformationAsExternal(
                    "unsafeMovesAllowed", "unsafeMoves", false, aspkb::Strategy::FALSIFY_OLD_VALUES);
            this->wm->integrateChanges();
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

    this->wm->playground->getAgentById(essentials::SystemConfig::getInstance()->getOwnRobotID())->updateObjective(obj);
    this->wm->integrateChanges();

    return obj;
    // return wumpus::model::Objective::EXPLORE;
}

/**
 * Determine Path and Actions (path is currently not used but might be interesting to know for future logging)
 * @return pair containing field coordinates of path and actions
 */
std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> PlanningModule::determinePathAndActions()
{
    std::string pathPredicate = "on";
    std::string actionsPredicate = "occurs";
    //    std::string movedInDangerPredicate = "movedInDanger"

    // TODO think of something more suitable
    auto startHorizon = 1;
    auto maxHorizon = maxHorizonFactor * this->wm->playground->getPlaygroundSize();

    std::string pathQueryValue = "holds(on(wildcard,wildcard), wildcard)";
    std::string actionsQueryValue = "occurs(wildcard,wildcard)";

    if (!this->pathAndActionsProblem) {

        this->pathAndActionsProblem = std::make_shared<aspkb::IncrementalProblem>(aspkb::TermManager::getInstance().getSolver(),
                std::vector<std::string>({pathQueryValue, actionsQueryValue}), std::map<std::string, std::string>(), KB_CONFIG_NAME,
                "pathAndActionsIncrementalProblem", "pathActions", startHorizon, maxHorizon, false);
    }

//    stepTerm->addRule("{occurs(A,t-1) : moveAction(A)} = 1."); //FIXME was in Extractor before (problems while loading from disk???)
#ifdef PM_DEBUG
    std::cout << "PlanningModule: Determining actions with incquery: Check if all agents are exhausted" << std::endl;
    this->determineAllAgentsExhausted();
#endif

    auto result = this->extractor->solveWithIncrementalExtensionQuery(this->pathAndActionsProblem);

    auto path = std::vector<std::pair<std::string, std::string>>();
    auto actions = std::vector<std::string>();

    if (!result.empty()) {
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Found actions " << std::endl;
#endif
        for (const auto& elem : result) {
            auto pathResult = wumpus::wm::PlanningModule::extractBinaryPredicateParameters(elem, "on");
            if (!pathResult.first.empty() && !pathResult.second.empty()) {
                path.emplace_back(pathResult.first, pathResult.second);
            }
            auto actionsResult = wumpus::wm::PlanningModule::extractBinaryPredicateParameters(elem, "occurs");
            if (!actionsResult.first.empty() && !actionsResult.second.empty()) {
                if (std::stoi(actionsResult.second) + 1 > actions.size()) {
                    actions.resize(std::stoi(actionsResult.second) + 1);
                }
#ifdef PM_DEBUG
                std::cout << "PlanningModule: adding action" << actionsResult.first << std::endl;
#endif
                actions[std::stoi(actionsResult.second)] = actionsResult.first;
            }
        }
    } else {
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Couldn't determine actions with incquery " << std::endl;
#endif
    }

    return std::make_pair(path, actions);
}

/**
 * the feedback of this has been moved into the knowledge base
 * @return
 */
bool PlanningModule::determineWumpusBlocksSafeMoves()
{

    // FIXME in possible next rule: use explored or visited????
    //    auto ret = this->extractor->extractReusableTemporaryQueryResult({"allPathsBlocked"}, "blockingWumpusExists",
    //            {"blockWumpus(X,Y) :- wumpusPossible(X,Y),fieldAdjacent(X,Y,A,B), not visited(A,B), stinky(C,D), fieldAdjacent(X,Y,C,D), visited(C,D).",
    //                    "blockWumpus(X,Y) :- wumpus(X,Y),fieldAdjacent(X,Y,A,B), not visited(A,B), stinky(C,D), fieldAdjacent(X,Y,C,D), visited(C,D).",
    //                    "possibleNext(X,Y) :- field(X,Y), not visited(X,Y), fieldAdjacent(A,B,X,Y), visited(A,B).",
    //                    "nextPossibleCount(N) :- N = #count{1,X,Y : possibleNext(X,Y)}.", "blockCount(M) :- M = #count{1,X,Y : blockWumpus(X,Y)}.",
    //                    "allPathsBlocked :- nextPossibleCount(M), blockCount(N), N == M."});
    auto ret = this->extractor->extractReusableTemporaryQueryResult({"allPathsBlocked"}, "blockingWumpusExists",
            {"wumpusPossibleOrDefinite(X,Y) :- wumpusPossible(X,Y).", "wumpusPossibleOrDefinite(X,Y) :- wumpus(X,Y).",
                    "exploredOrVisited(X,Y) :- explored(X,Y).", "exploredOrVisited(X,Y) :- visited(X,Y).", ":~ visited(X,Y) : possibleNext(X,Y). [1@1]",
                    "possibleNextCount(N) :- N = #count{1,X,Y : possibleNext(X,Y)}.", "possibleNext(X,Y) :- field(X,Y), not exploredOrVisited(X,Y), "
                                                                                      "fieldAdjacent(A,B,X,Y), exploredOrVisited(A,B), not trapPossible(X,Y), "
                                                                                      "not trap(X,Y).",
                    //                    "possibleNext(X,Y) :- field(X,Y), not visited(X,Y), fieldAdjacent(A,B,X,Y), visited(A,B), objective(A,
                    //                    moveToGoldField), me(A).",
                    "blockWumpus(X,Y) :- wumpusPossibleOrDefinite(X,Y), possibleNext(X,Y).",
                    "notAllPathsBlocked :- possibleNext(X,Y), not blockWumpus(X,Y), not objective(A, fetchOtherAgent), me(A).",
                    "notAllPathsBlocked :- me(A), objective(A,fetchOtherAgent), not blockWumpus(X,Y) : possibleNext(X,Y).", // FIXME review
                    "notAllPathsBlocked :- objective(A,shoot), me(A).", // rules only make sense in certain objectives. these have to be considered here because
                                                                        // the objective depends on the result of this operation
                    "notAllPathsBlocked :- objective(A,collectGold), me(A).",
                    //                    "notAllPathsBlocked :- objective(A,idle), me(A).", //FIXME idle here correct?
                    "notAllPathsBlocked :- not possibleNext(_,_)", "notAllPathsBlocked :- objective(A,leave), me(A).",
                    "allPathsBlocked :- field(_,_), not notAllPathsBlocked."}); // field(_,_) is a hack because parsing fails for some reason TODO investigate
                                                                                // why, probably expandRuleMP

    // FIXME something very wrong here. results for one query contained inquiry predicates from another... HACK
    bool found = false;
    for (const auto& r : ret) {
#ifdef PM_DEBUG
        std::cout << "PlanningModeule: R IN RET FOR DETERMINEWUMPUSBLOCKSSAFEMOVES: " << r << std::endl;
#endif
        if (r == "allPathsBlocked") {
            found = true;
        }
    }
#ifdef PM_DEBUG
    std::cout << "PlanningModule: blocking wumpus exists?: " << (found ? "True" : "False") << std::endl;
#endif
    //    wm->changeHandler->integrator->integrateInformationAsExternal("wumpusBlocksMoves", "wumpusMoves", found, aspkb::FALSIFY_OLD_VALUES);
    this->wm->playground->updateWumpusBlocksMoves(found);
    if (!found && this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->objective == wumpus::model::Objective::HUNT_WUMPUS) {
        this->wm->changeHandler->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        this->wm->changeHandler->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    }
    this->wm->integrateChanges();

    return found;
}

std::pair<std::pair<std::string, std::string>, std::string> PlanningModule::determinePosToShootFrom()
{
    auto position = std::pair<std::string, std::string>();
    auto heading = std::string();
    auto shotAt = std::vector<std::pair<std::string, std::string>>();
    auto result = this->extractor->extractReusableTemporaryQueryResult(
            {"targetPos(wildcard,wildcard)", "targetHeading(wildcard)", "fieldIsAhead(wildcard,wildcard)"}, "shootFrom", this->shootFromRules);

    // shooting wumpus should only be done from safe fields
    this->wm->changeHandler->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", false, aspkb::Strategy::INSERT_TRUE);
    this->wm->integrateChanges();
    if (!result.empty()) {

        for (const auto& res : result) {
#ifdef PM_DEBUG
            std::cout << "PlanningModule: Result for determinePosToShootFrom: " << std::endl;
            std::cout << res << ",";
#endif

            if (res.find("targetPos") != std::string::npos) {
                position = wumpus::wm::PlanningModule::extractBinaryPredicateParameters(res, "targetPos");
            } else if (res.find("targetHeading") != std::string::npos) {
                heading = wumpus::wm::PlanningModule::extractUnaryPredicateParameter(res);
            } else if (res.find("fieldIsAhead") != std::string::npos) {
                auto shotAtResult = wumpus::wm::PlanningModule::extractBinaryPredicateParameters(res, "fieldIsAhead");
                shotAt.emplace_back(shotAtResult.first, shotAtResult.second);
            }
        }
#ifdef PM_DEBUG
        std::cout << std::endl;
#endif

        std::stringstream ss;
        ss << "goal(" << position.first << ", " << position.second << ")";
        this->wm->changeHandler->integrator->integrateInformationAsExternal(ss.str(), "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        ss.str("");
        ss << "goalHeading(" << heading << ")";
        this->wm->changeHandler->integrator->integrateInformationAsExternal(ss.str(), "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
        ss.str("");

        if (this->shootingTargets->find(essentials::SystemConfig::getOwnRobotID()) != this->shootingTargets->end()) {
            this->shootingTargets->at(essentials::SystemConfig::getOwnRobotID()).clear();
        }
        for (const auto& field : shotAt) {
            this->addShootingTarget(essentials::SystemConfig::getOwnRobotID(), field);
        }

        this->wm->integrateChanges();
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

void PlanningModule::addShootingTarget(const int id, const std::pair<std::string, std::string>& shotAt)
{
    std::lock_guard<std::mutex> lock(this->shotAtMtx);
    if (this->shootingTargets->find(id) != this->shootingTargets->end()) {
        this->shootingTargets->at(id).insert(shotAt);
    }
    this->shootingTargets->emplace(id, std::set<std::pair<std::string, std::string>>({shotAt}));
}

/**
 * Activates a query with simple rules for getting the next action when no movement is required
 * @return
 */
std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> PlanningModule::determineActionsNoMovement()
{
    auto result = std::vector<std::string>();
    auto inquiry = "occurs(wildcard)";
    std::vector<std::string> rules;
    auto rule = "occurs(pickup) :- on(X,Y), glitter(X,Y)";
    rules.emplace_back(rule);
    rule = "occurs(shoot) :- on(X,Y), goal(X,Y), heading(H), goalHeading(H)";
    rules.emplace_back(rule);
    rule = "occurs(leave) :- on(X,Y), haveGold(A), me(A), initial(X,Y)";
    rules.emplace_back(rule);

    result = this->extractor->extractReusableTemporaryQueryResult({inquiry}, "simpleAction", rules);
    return std::make_pair<std::vector<std::pair<std::string, std::string>>>({}, result);
}

/**
 * Helper method to extract the (not nested) parameters from a binary predicate
 * @param str here: result string from a query
 * @param predName name of queried predicate
 * @return vector containing the two predicates or empty vector if predName wasn't found
 */
std::pair<std::string, std::string> PlanningModule::extractBinaryPredicateParameters(const std::string& str, const std::string& predName)
{
    std::string x;
    std::string y;
    auto start = str.find(predName);
    if (start == std::string::npos) {
        return std::pair<std::string, std::string>();
    }
    auto sep = str.find(',', start);
    if (sep == std::string::npos) {
        return std::pair<std::string, std::string>();
    }

    auto end = str.find(')');
    x = str.substr(start + predName.length() + 1, sep - start - predName.length() - 1);
    y = str.substr(sep + 1, end - sep - 1);
    return std::pair<std::string, std::string>{x, y};
}

/**
 *Helper method to get a simple unary parameter
 */
std::string PlanningModule::extractUnaryPredicateParameter(const std::string& str)
{
    auto braceStart = str.find('(');
    auto braceEnd = str.find(')');
    return str.substr(braceStart + 1, braceEnd - braceStart - 1);
}

/**
 * Load additional rules to be added to terms in the form of (Extension-)Queries. These files must only contiain the rules, no comments etc.
 * @param filePath (relative to the domain config folder of the workspace)
 * @param ruleContainer
 */
void PlanningModule::loadAdditionalRules(const std::string& filePath, std::vector<std::string>& ruleContainer)
{
    auto path2etc = std::getenv("DOMAIN_CONFIG_FOLDER");
    std::ifstream input(std::string(path2etc) + '/' + filePath);
    std::string line;
    while (std::getline(input, line)) {
        ruleContainer.push_back(line);
    }
}

bool PlanningModule::getIsPlanning()
{
    //    std::lock_guard<std::mutex> lock(this->planningMtx);
    return this->isPlanning;
}

void PlanningModule::setIsPlanning(bool planning)
{
#ifdef PM_DEBUG
    std::cout << "SET IS PLANNING: " << planning << std::endl;
#endif
    //    std::lock_guard<std::mutex> lock(this->planningMtx);
    this->isPlanning = planning;
}

bool PlanningModule::objectiveImpliesSimpleAction(const std::shared_ptr<model::Agent>& agent) const
{
    return agent->objective == model::COLLECT_GOLD || agent->objective == model::SHOOT || agent->objective == model::LEAVE;
}

bool PlanningModule::agentObjectiveRequiresGoal(const std::shared_ptr<wumpus::model::Agent>& agent) const
{
    return agent->objective == model::MOVE_TO_GOLD_FIELD || agent->objective == model::GO_HOME || agent->objective == model::Objective::FETCH_OTHER_AGENT; // ||
    //           agent->objective == model::Objective::HUNT_WUMPUS; // requires determinePosToShootFrom
}

bool PlanningModule::agentObjectiveRequiresMovement(const std::shared_ptr<wumpus::model::Agent>& agent) const
{
    return (agent->objective == model::GO_HOME || agent->objective == model::EXPLORE || agent->objective == model::MOVE_TO_GOLD_FIELD ||
            agent->objective == model::Objective::FETCH_OTHER_AGENT || agent->objective == model::Objective::HUNT_WUMPUS);
}

void PlanningModule::addBlockingWumpusOfOther(const std::pair<std::basic_string<char>, std::basic_string<char>>& wumpus)
{
    std::lock_guard<std::mutex> lock(this->wumpusMtxOther);
    this->blockingWumpiOfOthers.emplace_back(wumpus);
}

void PlanningModule::addBlockingWumpusOfSelf(const std::pair<std::basic_string<char>, std::basic_string<char>>& wumpus)
{
    std::lock_guard<std::mutex> lock(this->wumpusMtxSelf);
    this->blockingWumpiOfSelf.emplace_back(wumpus);
}

std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> PlanningModule::getShootingTargets()
{
    std::lock_guard<std::mutex> lock(this->shotAtMtx);
    return this->shootingTargets;
}

bool PlanningModule::determineAllAgentsExhausted()
{
    auto res = this->extractor->extractReusableTemporaryQueryResult({"allAgentsExhausted"}, "exhaustedAgents", this->allAgentsExhaustedRules);
    if (!res.empty()) {
        this->wm->changeHandler->integrator->integrateInformationAsExternal("everyAgentExhausted", "everyAgentExhausted", true, aspkb::Strategy::INSERT_TRUE);
#ifdef PM_DEBUG
        std::cout << "PlanningModule: All agents are exhausted!" << std::endl;
#endif
        return true;
    }
#ifdef PM_DEBUG
    std::cout << "PlanningModule: There are still agents who are not exhausted!" << std::endl;
#endif
    // TODO experimental
    this->wm->changeHandler->integrator->integrateInformationAsExternal("everyAgentExhausted", "everyAgentExhausted", false, aspkb::Strategy::INSERT_TRUE);
    return false;
}

struct StartEndRep
{ // TODO implement cleaner
    const std::string fromXRep = "x";
    const std::string fromYRep = "y";
    const std::string toXRep = "a";
    const std::string toYRep = "b";
    const std::string undef = "UNDEFINED";

    std::map<std::string, std::string> reps = {
            std::make_pair(fromXRep, undef), std::make_pair(fromYRep, undef), std::make_pair(toXRep, undef), std::make_pair(toYRep, undef)};
};

void PlanningModule::checkSafePathsExistsForOtherAgents(std::pair<int, int> from, const std::map<int, std::pair<int, int>>& tos)
{
    // assume correct placement TODO replace with struct or similar?
    auto startHorizon = 1;
    auto maxHorizon = maxHorizonFactor * this->wm->playground->getPlaygroundSize();
    std::vector<std::string> result;

    for (const auto& to : tos) {

        // trivial case - incremental approach doesn't work for this so far
        if (from.first == to.second.first && from.second == to.second.first) {
            result.push_back("start(" + std::to_string(from.first) + "," + std::to_string(from.second) + (")"));
            result.push_back("end(" + std::to_string(to.second.first) + "," + std::to_string(to.second.second) + (")"));
        } else {

            auto startEndRep = StartEndRep();
            startEndRep.reps.at(startEndRep.fromXRep) = std::to_string(from.first);
            startEndRep.reps.at(startEndRep.fromYRep) = std::to_string(from.second);
            startEndRep.reps.at(startEndRep.toXRep) = std::to_string(to.second.first);
            startEndRep.reps.at(startEndRep.toYRep) = std::to_string(to.second.second);

            auto externalPrefix =
                    "safePath" + std::to_string(from.first) + std::to_string(from.second) + std::to_string(to.second.first) + std::to_string(to.second.second);

            // create necessary problems if necessary
            if (this->safePathExistsForOtherAgentProblems.find(from) != this->safePathExistsForOtherAgentProblems.end()) {
                auto problemForStart = this->safePathExistsForOtherAgentProblems.at(from);

                if (problemForStart.find(to.second) == problemForStart.end()) {
                    std::cout << "Found from " << from.first << ", " << from.second << " but couldn't find to! " << to.second.first << ", " << to.second.second
                              << std::endl;
                    auto problem = std::make_shared<aspkb::IncrementalProblem>(aspkb::TermManager::getInstance().getSolver(),
                            std::vector<std::string>({"pathComplete(wildcard)"}), startEndRep.reps, KB_CONFIG_NAME, "safePathExistsIncrementalProblem",
                            externalPrefix, startHorizon, maxHorizon, false);

                    this->safePathExistsForOtherAgentProblems.at(from).emplace(to.second, problem);
                } else {
                    std::cout << "Found from  " << from.first << ", " << from.second << " and to!" << to.second.first << ", " << to.second.second << std::endl;
                }
            } else {
                std::cout << "Couldn't find from! " << from.first << ", " << from.second << "for to " << to.second.first << ", " << to.second.second
                          << std::endl;

                auto map = std::map<std::pair<int, int>, std::shared_ptr<aspkb::IncrementalProblem>>();
                auto problem = std::make_shared<aspkb::IncrementalProblem>(aspkb::TermManager::getInstance().getSolver(),
                        std::vector<std::string>({"pathComplete(wildcard)"}), startEndRep.reps, KB_CONFIG_NAME, "safePathExistsIncrementalProblem",
                        externalPrefix, startHorizon, maxHorizon, false);
                map.emplace(to.second, problem);
                this->safePathExistsForOtherAgentProblems.emplace(from, map);
            };

            std::string safePathQueryValue = "path(wildcard, wildcard, wildcard)";
            std::cout << "solving for from " << from.first << ", " << from.second << ", to:  " << to.second.first << ", " << to.second.second << std::endl;
            for (const auto& elem : this->safePathExistsForOtherAgentProblems) {
                std::cout << "problems map from: " << elem.first.first << ", " << elem.first.second << std::endl;
                for (const auto& innerElem : elem.second) {
                    std::cout << "problems map to: " << innerElem.first.first << ", " << innerElem.first.second << std::endl;
                }
            }

            // set externals for start
            std::stringstream ss;
            ss << "start(" << from.first << ", " << from.second << ")";
            auto start = ss.str();
            this->wm->changeHandler->integrator->integrateInformationAsExternal(start, "safePathStart", true, aspkb::Strategy::INSERT_TRUE);
            ss.str("");
            ss << "end(" << to.second.first << ", " << to.second.second << ")";
            auto end = ss.str();
            this->wm->changeHandler->integrator->integrateInformationAsExternal(end, "safePathEnd", true, aspkb::Strategy::INSERT_TRUE);
            this->wm->integrateChanges();

            // try to get result
            result = this->extractor->solveWithIncrementalExtensionQuery(this->safePathExistsForOtherAgentProblems.at(from).at(to.second));

            // reset externals
            this->wm->changeHandler->integrator->integrateInformationAsExternal(start, "safePathStart", false, aspkb::Strategy::INSERT_TRUE);
            this->wm->changeHandler->integrator->integrateInformationAsExternal(end, "safePathEnd", false, aspkb::Strategy::INSERT_TRUE);
            this->wm->integrateChanges();

            auto path = std::vector<std::pair<std::string, std::string>>();
        }

        if (!result.empty()) {
            std::cout << "PlanningModule: SAFEPATHEXISTS: result not empty " << std::endl;
            std::stringstream ss2;
            ss2 << "safePathExists(" << from.first << "," << from.second << "," << to.second.first << "," << to.second.second << ")";
            auto finalRep = ss2.str();
            std::cout << "SAFEPATHEXISTS: " << finalRep << std::endl;
            this->wm->playground->getAgentById(to.first)->updateHaveSafePathToGold();
            this->wm->changeHandler->integrator->integrateInformationAsExternal(finalRep, "safePathExists", true, aspkb::Strategy::INSERT_TRUE);
            this->wm->changeHandler->integrator->applyChanges();
        } else {
            //            std::cout << "PlanningModule: Should have a safe path for the first world" << std::endl;
            //            throw std::exception();
        }
    }
    // FIXME TODO implement handling result / setting of external etc
    //    throw std::exception();
}

std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> PlanningModule::tryDetermineActionsByObjective(
        const std::shared_ptr<wumpus::model::Agent>& agent)
{
    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> fieldsActionsPair;

    if (agentObjectiveRequiresMovement(agent)) {

        if (agentObjectiveRequiresGoal(agent)) {
            auto goal = this->determineGoal();
            if (goal.empty()) { // situation: agent wants to fetch other agent but cannot find a valid goal which means a wumpus or trap is blocking its way. if
                                // it has an arrow, it should try to shoot
                // as many wumpi blocking both itself and others (might require new asp rules?) or if a trap is blocking it could go home already? TODO review
                if (this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->hasArrow && this->determineWumpusBlocksSafeMoves()) {
                    this->determinePosToShootFrom();
                } else { // FIXME hack
                    auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());
                    if (localAgent->currentPosition->x == localAgent->initialPosition->x && localAgent->currentPosition->y == localAgent->initialPosition->y) {
                        return std::make_pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>({}, {"leave"});
                    } else {
                        localAgent->updateObjective(model::Objective::GO_HOME);
                        this->wm->integrateChanges();
                        this->determineGoal();
                    }
                }
            }
        } else if (agent->objective == model::Objective::HUNT_WUMPUS) {
            auto target = this->determinePosToShootFrom();
            auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());
            std::cout << "PlanningModule: TARGET " << target.first.first << ", " << target.first.second << " with heading " << target.second << std::endl;
            if (localAgent->currentPosition->x == std::stoi(target.first.first) && localAgent->currentPosition->y == std::stoi(target.first.second) &&
                    localAgent->currentHeading == std::stoi(target.second)) {
                return std::make_pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>({}, {"shoot"});
            }
        }

        //        this->determineAllAgentsExhausted(); // FIXME experimental
        fieldsActionsPair = tryGetSafeActions(agent);

        if (fieldsActionsPair.second.empty()) {
#ifdef PM_DEBUG
            std::cout << "PlanningModule: exhausted options 1!" << std::endl;
#endif
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
            this->wm->integrateChanges();
            this->determineObjective();
            auto possiblyNewActions = tryDetermineActionsByObjective(agent); // tryGetSafeActions(agent);// <- objective might have changed here!
            if (!possiblyNewActions.second.empty()) {
                return possiblyNewActions;
            }
            return std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>();
        }

    } else if (objectiveImpliesSimpleAction(agent)) {
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Objective implies simple action" << std::endl;
#endif
        fieldsActionsPair = this->determineActionsNoMovement();

    } else if (agent->objective == wumpus::model::Objective::IDLE) {
#ifdef PM_DEBUG
        std::cout << "PlanningModule: Objective implies idling!" << std::endl;
#endif
        this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(true);
        fieldsActionsPair = std::make_pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>>(
                {}, std::vector<std::string>({(random() % 2 == 0 ? "turnLeft" : "turnRight")}));
    }
    return fieldsActionsPair;
}

} /* namespace wm */
} /* namespace wumpus*/