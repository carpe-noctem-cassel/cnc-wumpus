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
    std::stringstream ss;
    ss << "agent(" << std::to_string(id) << ")" << std::endl;
    this->integrator->integrateAsTermWithProgramSection(
            "wumpus_agent", std::make_pair<std::vector<std::string>, std::vector<std::string>>({"n"}, {std::to_string(id)}));
    this->integrator->integrateInformationAsExternal(ss.str(), "wumpus_agent", true, aspkb::Strategy::INSERT_TRUE);
    ss.str("");
    if (me) {
        ss << "me(" << std::to_string(id) << ")";
        this->integrator->integrateInformationAsExternal(ss.str(), "me", true, aspkb::Strategy::INSERT_TRUE);
    }
}

void ChangeHandler::unregisterAgent(int id)
{
    std::stringstream ss;
    ss << "agent(" << std::to_string(id) << ")" << std::endl;
    this->integrator->integrateInformationAsExternal(ss.str(), "wumpus_agent", false, aspkb::Strategy::INSERT_TRUE);
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
    this->integrator->integrateInformationAsExternal(ss.str(), "stinky", field->stinky, aspkb::Strategy::INSERT_TRUE);
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

void ChangeHandler::handleSetDiedOn(std::shared_ptr<wumpus::model::Field> field)
{
    std::stringstream ss;
    ss << "otherAgentDiedOn(" << field->x << field->y << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "diedOn", true, aspkb::Strategy::INSERT_TRUE);
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
    this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", false, aspkb::Strategy::INSERT_TRUE);
}

void ChangeHandler::handleChangedShotAt(int id, std::shared_ptr<wumpus::model::Field> field)
{
    this->integrator->integrateInformationAsExternal(
            "shotAt(" + std::to_string(field->x) + "," + std::to_string(field->y) + ")", "shotAt", true, aspkb::Strategy::INSERT_TRUE);

    if (id == essentials::SystemConfig::getOwnRobotID()) {
        this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMovesAllowed", false, aspkb::Strategy::INSERT_TRUE);
    }

    // a blocking wumpus for the local agent might have been shot
    // TODO check if shot at field contained a blocking wumpus for the local agent
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(false);
}

void ChangeHandler::handleChangedExplored(std::shared_ptr<wumpus::model::Field> field)
{
    this->integrator->integrateInformationAsExternal(
            "explored(" + std::to_string(field->x) + "," + std::to_string(field->y) + ")", "explored", true, aspkb::Strategy::INSERT_TRUE);

    // a new field has been explored. if local agent is exhausted, this might open up new possibilities
    // TODO: improve this by only considering "problematic" fields
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(false);
}

// RESET GOAL
void ChangeHandler::handleScream()
{
    std::cout << "CHANGEHANDLER : HANDLE SCREAM ************************************" << std::endl;

    this->handleShotAtFields();
    this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    this->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    // TODO does this make sense?
    this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", false, aspkb::Strategy::INSERT_TRUE);
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(false);

    for (const auto& fieldsById : *this->wm->getShotAtFields()) {
        if (this->wm->playground->getAgentById(fieldsById.first)->shot) {
            for (const auto& shotAt : fieldsById.second) {
                //            iff agent actually shot
                std::cout << "shotAt: " << shotAt.first << "," << shotAt.second << std::endl;
                // FIXME don't consider shot at fields
                for (const auto& affected : this->wm->playground->getAdjacentFields(std::stoi(shotAt.first), std::stoi(shotAt.second))) {
                    std::cout << "affected: " << affected->x << ", " << affected->y << std::endl;
                    this->wm->playground->getField(affected->x, affected->y)->updateExplored(false);
                    this->wm->playground->getField(affected->x, affected->y)->updateVisited(false);
                    // FIXME ?
                    this->wm->playground->getField(affected->x, affected->y)->updateStinky(false);
                }
            }
        }
    }
}

void ChangeHandler::handleSilence()
{
    std::cout << "CHANGEHANDLER : HANDLE SILENCE ************************************" << std::endl;
    this->handleShotAtFields();
    this->integrator->integrateInformationAsExternal("", "goal", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    this->integrator->integrateInformationAsExternal("", "goalHeading", true, aspkb::Strategy::FALSIFY_OLD_VALUES);
    // TODO does this make sense?
    this->integrator->integrateInformationAsExternal("unsafeMovesAllowed", "unsafeMoves", false, aspkb::Strategy::INSERT_TRUE);
    this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID())->updateExhausted(false);
}

void ChangeHandler::handleShotAtFields() const
{
    std::stringstream ss;
    auto agentsWhoShot = this->wm->playground->getAgentsWhoShot();
    for (auto a : agentsWhoShot) {
        std::cout << "  AGENTS WHO SHOT: " << a->id << std::endl;
    }
    auto shotAtFields = wm->getShotAtFields();
    for (const auto& agent : *shotAtFields) {
        for (auto f : agent.second) {
            std::cout << "  FIELD SHOT AT: " << f.first << ", " << f.second << std::endl;
        }
    }
    for (const auto& agent : agentsWhoShot) {
        if (shotAtFields->find(agent->id) == shotAtFields->end()) {
            std::cout << "No shooting targets for id " << agent->id << "!" << std::endl;
            throw std::exception(); // FIXME remove, only for debugging
            return;
        } else {
            for (const auto& shotAt : shotAtFields->at(agent->id)) {
                ss << "shotAt(" << shotAt.first << "," << shotAt.second << ")";
                wm->changeHandler->integrator->integrateInformationAsExternal(ss.str(), "shotAt", true, aspkb::INSERT_TRUE);
                ss.str("");
            }
        }
    }
}

void ChangeHandler::handleTurn(long turn)
{
    // TODO necessary?
}

void ChangeHandler::handleChangedExhausted(int id, bool exhausted)
{
    std::stringstream ss;
    ss << "exhausted (" << id << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "exhausted", exhausted, aspkb::Strategy::INSERT_TRUE);
}

void ChangeHandler::handleChangedShot(int agentId)
{
    std::stringstream ss;
    ss << "shot(" << std::to_string(agentId) << ")";
    this->integrator->integrateInformationAsExternal(ss.str(), "agentsWhoShot", true, aspkb::Strategy::INSERT_TRUE);
}

} /* namespace wm*/

} /* namespace wumpus*/
