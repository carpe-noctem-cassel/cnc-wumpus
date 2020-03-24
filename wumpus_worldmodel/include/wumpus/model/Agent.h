#pragma once

#include "Objective.h"
#include <wumpus/model/DomainElement.h>
#include <memory>

namespace wumpus
{
class WumpusWorldModel;
namespace wm
{
class ChangeHandler;
}
namespace model
{
class Field;
class Agent : public wumpus::model::DomainElement
{
public:
    Agent(wumpus::wm::ChangeHandler* ch, int agentId);
    virtual ~Agent() = default;
    std::shared_ptr<Field> initialPosition;
    std::shared_ptr<Field> currentPosition;
    std::shared_ptr<Field> moveGoal;
    Objective objective;
    int currentHeading;
    int turn;
    bool hasArrow;
    bool hasGold;
    bool exited;
    bool shot;
    std::shared_ptr<wumpus::model::Field> diedOn;
    bool exhausted;
    bool replanNecessary;
    int id;

    void registerAgent(bool me);
    void unregisterAgent();
    void updateCurrentMoveGoal(std::shared_ptr<Field> goal);
    void updatePosition(std::shared_ptr<Field> field);
    void updateInitialPosition(std::shared_ptr<Field> field);
    void updateObjective(Objective obj);
    void updateHeading(int heading);
    void updateArrow(bool arrow);
    void updateHaveGold(bool gold);
    void updateExhausted(bool exhausted);
    void updateShot();

    void setExited();
    void setDead(const std::shared_ptr<wumpus::model::Field>& diedOn);

private:
    bool registered;
};

} /* namespace model */
} /* namespace wumpus */
