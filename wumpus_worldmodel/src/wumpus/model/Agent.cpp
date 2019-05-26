#include "wumpus/model/Agent.h"
#include <utility>
#include <wumpus/WumpusWorldModel.h>

namespace wumpus
{
namespace model
{

Agent::Agent(wumpus::wm::ChangeHandler* ch, int agentId)
        : DomainElement(ch)
        , id(agentId)
{
    this->replanNecessary = true;
    this->initialPosition = nullptr;
    this->currentPosition = nullptr;
    this->moveGoal = nullptr;
    this->currentHeading = -1;
    this->hasArrow = false;
    this->hasGold = false;
    this->registered = false;
    this->objective = Objective::UNDEFINED;
}

void Agent::registerAgent(bool me)
{
    if (!this->registered) {

        this->ch->registerNewAgent(this->id, me);
        this->registered = true;
    }
}

void Agent::updateCurrentMoveGoal(std::shared_ptr<wumpus::model::Field> goal)
{
    if (!this->moveGoal || this->moveGoal != goal) {
        this->moveGoal = goal;
        this->ch->handleChangedMoveGoal(this->id, goal);
    }
}

void Agent::updateInitialPosition(std::shared_ptr<wumpus::model::Field> field)
{
    if (!this->initialPosition || field != this->initialPosition) {
        this->initialPosition = field;
        this->ch->handleSetInitialPosition(field);
    }
}

void Agent::updatePosition(std::shared_ptr<wumpus::model::Field> field)
{
    if (!this->currentPosition || field != this->currentPosition) {
        this->currentPosition = field;
        this->ch->handleChangedPosition(field);
        this->ch->clearBlacklist();
    }

    if (this->moveGoal && field == this->moveGoal) {
        //std::cout << "GOAL REACHED!!!" << std::endl;
        this->ch->handleGoalReached(this->id);
        this->moveGoal.reset();
    }
}

void Agent::updateHeading(int heading)
{
    if (this->currentHeading < 0 || this->currentHeading != heading) {
        this->currentHeading = heading;
        this->ch->handleChangedHeading(heading);
    }
}

void Agent::updateHaveGold(bool gold)
{
    if (this->hasGold != gold) {
        this->hasGold = gold;
        this->ch->handleSetGold(this->id);
    }
}

void Agent::updateArrow(bool arrow)
{
    if (arrow != this->hasArrow) {
        this->hasArrow = arrow;
        this->ch->handleChangedArrow(this->id, arrow);
    }
}

void Agent::updateObjective(Objective obj) {
    if(this->objective != obj) {
        std::cout << "AGENT: obj changed" << std::endl;
        this->objective = obj;
        this->ch->handleChangedObjective(this->id, this->objective);
    }
}

} /* namespace model */
} /* namespace wumpus */
