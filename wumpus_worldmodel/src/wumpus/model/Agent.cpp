#include "wumpus/model/Agent.h"
#include <memory>
#include <utility>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/model/Field.h>

namespace wumpus
{
namespace model
{

Agent::Agent(wumpus::wm::ChangeHandler* ch, int agentId)
        : DomainElement(ch)
        , id(agentId)
        , shot(false)
        , hasArrow(false)
        , hasGold(false)
        , hasSafePathToGold(false)
        , registered(false)
        , exited(false)
        , exhausted(false)
        , replanNecessary(true)
        , initialPosition(nullptr)
        , currentPosition(nullptr)
        , moveGoal(nullptr)
        , diedOn(nullptr)
        , currentHeading(-1)
        , turn(1)
        , objective(Objective::UNDEFINED)

{
}

/**
 * Adds knowledge of the existence of this agent to the knowledgebase, including whether this is the local agent.
 * @param me
 */
void Agent::registerAgent(bool me)
{
    if (!this->registered) {

        this->ch->registerNewAgent(this->id, me);
        this->registered = true;
    }
}

/**
 * Agent should be unregistered as soon as it died or exited.
 */
void Agent::unregisterAgent()
{

    wumpus::WumpusWorldModel::getInstance()->playground->removeAgent(this->id);
    // TODO necessary?
    this->registered = false;
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
        //        if (this->currentPosition) {
        this->currentPosition = field;
        this->ch->handleChangedPosition(this->id, field);
        //        }
    }
    if (this->moveGoal && field == this->moveGoal) {
        std::cout << "GOAL REACHED!!!" << std::endl;
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

void Agent::updateObjective(Objective obj)
{
    if (this->objective != obj) {
        this->objective = obj;
        this->ch->handleChangedObjective(this->id, this->objective);
    }
}

void Agent::setDead(const std::shared_ptr<wumpus::model::Field>& field)
{
    std::cout << "Agent: set dead " << std::endl;
//    throw std::exception();
    if (!this->diedOn || this->diedOn != field) {
        this->diedOn = field;
        this->ch->handleSetDiedOn(field);
        this->ch->unregisterAgent(this->id);
    }
//    throw std::exception();
}

void Agent::setExited()
{
    this->exited = true;
    this->ch->unregisterAgent(this->id);
}

void Agent::updateExhausted(bool exhausted)
{
    if (this->exhausted != exhausted) {
        std::cout << "WumpusAgent: Changed exhausted of " << this->id << " from " << (this->exhausted ? "True" : "False") << " to "
                  << (exhausted ? "True" : "False") << std::endl;
        this->exhausted = exhausted;
        this->ch->handleChangedExhausted(this->id, exhausted);
    }
}

void Agent::updateShot()
{
    if (!this->shot) {
        this->shot = true;
        this->ch->handleChangedShot(this->id);
    }
}

// should only have to be set to true once
void Agent::updateHaveSafePathToGold()
{
    if (!this->hasSafePathToGold) {
        this->hasSafePathToGold = true;
        this->ch->handleChangedSafeGoldPath(this->id);
    }
}

} /* namespace model */
} /* namespace wumpus */
