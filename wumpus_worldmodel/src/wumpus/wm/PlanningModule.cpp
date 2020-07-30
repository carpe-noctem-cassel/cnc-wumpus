#include "wumpus/wm/PlanningModule.h"
#include "wumpus/WumpusWorldModel.h"
#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <aspkb/Extractor.h>
#include <aspkb/IncrementalProblem.h>
#include <aspkb/PossibleNextFieldGenerationProblem.h>
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
#include <wumpus/wm/util/PlannerUtils.h>

#define PM_DEBUG
namespace wumpus
{
namespace wm
{
std::mutex PlanningModule::planningMtx;
PlanningModule::PlanningModule(wumpus::WumpusWorldModel* wm, aspkb::Extractor* extractor, aspkb::Integrator* integrator)
        : wm(wm)
        , integrator(integrator)
        , pathActionsPlanner(extractor, integrator)
        , blockEval(extractor, integrator)
        , objEval(extractor, integrator)
        , exhaustionEval(extractor, integrator)
{

    this->isPlanning = false;
    auto sc = essentials::SystemConfig::getInstance();
    this->communicationAllowed = (*sc)["WumpusWorldModel"]->get<bool>("Agents.allowCommunication", NULL);
}

PlanningModule::~PlanningModule()
{
}

/**
 *
 * @param agent
 * @return
 */
std::pair<int, std::vector<WumpusEnums::actions>> PlanningModule::processNextActionRequest(std::shared_ptr<wumpus::model::Agent> agent)
{
    std::lock_guard<std::mutex> lock(PlanningModule::planningMtx);
    this->integrator->applyChanges();
    this->wm->wumpusSimData.setIntegratedFromSimulator(false);
    this->setIsPlanning(true);

    auto actions = std::vector<WumpusEnums::actions>();
    auto params = std::vector<std::string>();
    std::stringstream ss;

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> fieldsActionsPair;
    std::string goal;

    // FIXME does this make sense here?

    //    if(!this->wm->playground->goldFieldKnown) {
    auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());
    //    std::cout << "PlanningModule: isBlocked: " << localAgent->isBlockedByWumpus() << ", hasgold: " << localAgent->hasGold << std::endl;

    if (this->communicationAllowed) {
        bool foundUnidling = false;
        for (const auto& id : this->wm->getAgentIDsForExperiment()) {
            auto otherAgent = this->wm->playground->getAgentById(id);
            if(otherAgent) {
                if (otherAgent->objective != wumpus::model::Objective::IDLE) {
                    foundUnidling = true;
                }
            }

        }
        if (localAgent->objective == wumpus::model::Objective::IDLE && !localAgent->replanNecessary && foundUnidling) {
            std::cout << "Agent was idling before and no new insights have happened!" << std::endl;
            actions.emplace_back(WumpusEnums::turnLeft);
            return std::make_pair(localAgent->id, actions);
        }
    }

    if (localAgent->replanNecessary && (localAgent->isBlockedByTrap() || localAgent->isBlockedByWumpus())) { //  &&
        // localAgent->hasGold /TODO gold hack as this breaks hunting wumpi for other agents
        this->blockEval.determineBlockingElementsForSelf();
    }
    //    }

    // determine objective - might cause replan necessary to be set
    //    if (agent->replanNecessary) {
    if (this->wm->wumpusSimData.communicationAllowed) {

        this->exhaustionEval.determineAllAgentsExhausted(); // experimental TODO
    }

    auto obj = this->objEval.determineObjective();
    //    }

    // determine actions
    if (agent->replanNecessary || this->lastPathAndActions.second.empty()) { // FIXME this seems to be problematic. make sure replan necessary is set
        //    correctly for all occasions
        do {
            std::cout << "~~~~~~~~~~~~~~~~~~~ PLANNING " << std::endl;
            this->pathActionsPlanner.timePlanningStart = this->wm->getEngine()->getAlicaClock()->now();
            fieldsActionsPair = this->pathActionsPlanner.tryDetermineActionsByObjective(agent);

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
            this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
            this->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
            std::cout << "SHOT!" << std::endl;
            this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateShot();
            this->integrator->applyChanges();
        } else if (tmp.find("pickup") != std::string::npos) {
            actionArray[index] = WumpusEnums::pickUpGold;
        } else if (tmp.find("leave") != std::string::npos) {
            actionArray[index] = WumpusEnums::leave;
            this->pathActionsPlanner.extractor->writeGetSolutionStatsReusable("", -1, -1);
            this->pathActionsPlanner.extractor->timesWritten++;
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

} /* namespace wm */
} /* namespace wumpus*/