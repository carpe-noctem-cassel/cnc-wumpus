#include "wumpus/wm/ChangeHandler.h"
#include <aspkb/Integrator.h>
#include <aspkb/Strategy.h>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/model/Agent.h>
#include <wumpus/model/Field.h>
namespace wumpus
{
namespace wm
{

ChangeHandler::ChangeHandler(wumpus::WumpusWorldModel* wm)
        : wm(wm)
{
    this->integrator = new aspkb::Integrator();
}

ChangeHandler::~ChangeHandler()
{
    delete this->integrator;
}

void ChangeHandler::registerNewAgent(int id, bool me)
{
    this->integrator->integrateAsTermWithProgramSection(
            "wumpus_agent", std::make_pair<std::vector<std::string>, std::vector<std::string>>({"n"}, {std::to_string(id)}));
    if (me) {
        std::stringstream ss;
        ss << "me(" << std::to_string(id) << ")";
        this->integrator->integrateInformationAsExternal(ss.str(), "me", true, aspkb::Strategy::INSERT_TRUE);
    }
}

void ChangeHandler::handleChangedPosition(std::shared_ptr<wumpus::model::Field> field)
{
    std::stringstream ss;
    ss << "on(" << field->x << ", " << field->y << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "position", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
}

void ChangeHandler::handleChangedHeading(int newHeading)
{

    std::stringstream ss;
    ss << "heading(" << newHeading << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "heading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
}

void ChangeHandler::handleChangedStench(std::shared_ptr<wumpus::model::Field> field)
{

    std::stringstream ss;
    ss << "stinky(" << field->x << "," << field->y << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "stinky", true, aspkb::Strategy::INSERT_TRUE);
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->replanNecessary = true;
}

void ChangeHandler::handleSetGold(int agentId)
{
    std::stringstream ss;
    ss << "haveGold(" << agentId << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "haveGold", true, aspkb::Strategy::INSERT_TRUE);
}

void ChangeHandler::handleSetInitialPosition(std::shared_ptr<wumpus::model::Field> field)
{
    std::stringstream ss;
    ss << "initial(" << field->x << ", " << field->y << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "initial", true, aspkb::Strategy::INSERT_TRUE);
}

void ChangeHandler::handleSetDrafty(std::shared_ptr<wumpus::model::Field> field)
{
    std::stringstream ss;
    ss << "drafty(" << field->x << ", " << field->y << ")";
    auto str = ss.str();
    this->integrator->integrateInformationAsExternal(str, "draft", true, aspkb::Strategy::INSERT_TRUE);

    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->replanNecessary = true;
}

void ChangeHandler::handleSetGlitter(std::shared_ptr<wumpus::model::Field> field)
{
    std::stringstream ss;
    ss << "glitter(" << field->x << ", " << field->y << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "glitter", true, aspkb::Strategy::INSERT_TRUE);
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->replanNecessary = true;
}

void ChangeHandler::handleChangedArrow(int agentId, bool arrow)
{
    // std::cout << "change handler: updating arrow!" << std::endl;
    std::stringstream ss;
    if (arrow) {

        ss << "arrow(" << agentId << ")";
    }

    this->integrator->integrateInformationAsExternal(ss.str(), "arrow", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
}

void ChangeHandler::handleChangedObjective(int id, wumpus::model::Objective objective)
{
    std::stringstream ss;
    ss << "objective(" << id << ", ";

    switch (objective) {

    case wumpus::model::Objective::EXPLORE:
        ss << "explore";
        break;
    case wumpus::model::Objective::GO_HOME:
        ss << "goHome";
        break;
    case wumpus::model::Objective::COLLECT_GOLD:
        ss << "collectGold";
        break;
    case wumpus::model::Objective::HUNT_WUMPUS:
        ss << "huntWumpus";
        break;
    case wumpus::model::Objective::SHOOT:
        ss << "shootWumpus";
        break;
    case wumpus::model::Objective::LEAVE:
        ss << "leaveCave";
        break;
    case wumpus::model::Objective::MOVE_TO_GOLD_FIELD:
        ss << "moveToGoldField";
        break;
    case wumpus::model::Objective::IDLE:
        ss << "idle";
        break;
    }
    ss << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "objective", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    if (id == essentials::SystemConfig::getOwnRobotID()) {
        this->wm->playground->getAgentById(id)->replanNecessary = true;
    }
}

/**
 * Knowledge about the field size is necessary for defining externals.
 * When externals are defined, background knowledge can be loaded.
 * @param fieldSize
 */
void ChangeHandler::handleSetFieldSize(int fieldSize)
{
    this->integrator->integrateAsTermWithProgramSection(
            "wumpus_playground", std::make_pair<std::vector<std::string>, std::vector<std::string>>({"n"}, {std::to_string(fieldSize)}));
    this->integrator->integrateAsTermWithProgramSection("wumpus_externals", {});
    this->integrator->integrateAsTermWithProgramSection("wumpus_stenchPerception", {});
    this->integrator->integrateAsTermWithProgramSection("wumpus_draftPerception", {});
    // this->integrator->integrateAsTermWithProgramSection("wumpus_objective",{});
}

void ChangeHandler::handleChangedMoveGoal(int id, std::shared_ptr<wumpus::model::Field> goal)
{
    if (goal) {
        std::stringstream ss;
        ss << "goal(" << goal->x << ", " << goal->y << ")";
        this->integrator->integrateInformationAsExternal(ss.str(), "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    } else {
        this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    }
}

void ChangeHandler::handleGoalReached(int id)
{
    this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
}

void ChangeHandler::handleChangedVisited(std::shared_ptr<wumpus::model::Field> field)
{
    this->integrator->integrateInformationAsExternal(
            "visited(" + std::to_string(field->x) + "," + std::to_string(field->y) + ")", "visited", true, aspkb::Strategy::INSERT_TRUE);

    // unsafe moves are no longer allowed when an unvisited field has been vistied TODO only when own agent explored it?
    this->wm->changeHandler->integrator->integrateInformationAsExternal("", "unsafeMoves", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
}

void ChangeHandler::handleChangedShotAt(std::shared_ptr<wumpus::model::Field> field)
{
    this->integrator->integrateInformationAsExternal(
            "shotAt(" + std::to_string(field->x) + "," + std::to_string(field->y) + ")", "shotAt", true, aspkb::Strategy::INSERT_TRUE);

    this->wm->changeHandler->integrator->integrateInformationAsExternal("", "unsafeMoves", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
}

void ChangeHandler::handleChangedExplored(std::shared_ptr<wumpus::model::Field> field)
{
    this->integrator->integrateInformationAsExternal(
            "explored(" + std::to_string(field->x) + "," + std::to_string(field->y) + ")", "explored", true, aspkb::Strategy::INSERT_TRUE);
}

// RESET GOAL
void ChangeHandler::handleScream()
{
    this->wm->changeHandler->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    this->wm->changeHandler->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    // TODO does this make sense?
    this->wm->changeHandler->integrator->integrateInformationAsExternal("", "unsafeMoves", true, aspkb::Strategy::FALSIFY_OLD_VALUES);

    for (auto shotAt : this->wm->getShotAtFields()) {
        std::cout << "shotAt: " << shotAt.first << "," << shotAt.second << std::endl;
        // FIXME don't consider shot at fields
        for (auto affected : this->wm->playground->getAdjacentFields(std::stoi(shotAt.first), std::stoi(shotAt.second))) {
            std::cout << "affected: " << affected->x << ", " << affected->y << std::endl;
            std::stringstream ss;
            ss << "explored(" << affected->x << "," << affected->y << ")";
            this->wm->changeHandler->integrator->integrateInformationAsExternal(ss.str(), "visited", false, aspkb::Strategy::INSERT_TRUE);
            std::cout << "setting " << affected->x << ", " << affected->y << "to unvisited" << std::endl;
            this->wm->playground->getField(affected->x, affected->y)->updateVisited(false);
            ss.str("");
            ss << "stinky(" << affected->x << "," << affected->y << ")";
            this->wm->changeHandler->integrator->integrateInformationAsExternal(ss.str(), "stench", false, aspkb::Strategy::INSERT_TRUE);
            // FIXME ?
            //            this->wm->playground->getField(affected->x, affected->y)->updateStinky(false);
        }
    }
}

void ChangeHandler::handleSilence()
{
    this->wm->changeHandler->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    this->wm->changeHandler->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    // TODO does this make sense?
    this->wm->changeHandler->integrator->integrateInformationAsExternal("", "unsafeMoves", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
}
void ChangeHandler::handleTurn(long turn)
{
    // TODO necessary?
}

} /* namespace wm*/

} /* namespace wumpus*/
