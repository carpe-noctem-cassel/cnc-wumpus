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
    int id;
    std::shared_ptr<Field> initialPosition;
    std::shared_ptr<Field> currentPosition;
    std::shared_ptr<Field> moveGoal;
    std::unordered_set<std::shared_ptr<Field>> fieldsWithBlockingWumpi;
    std::unordered_set<std::shared_ptr<Field>> fieldsWithBlockingTraps;
    std::shared_ptr<wumpus::model::Field> diedOn;
    int currentHeading;
    int turn;
    bool hasArrow;
    bool hasGold;
    bool exited;
    bool shot;
    bool exhausted;
    bool replanNecessary;
    bool unsafeMovesAllowed;
    bool hasSafePathToGold;
    Objective objective;


    void registerAgent(bool me);
    void unregisterAgent();
    void updateCurrentMoveGoal(std::shared_ptr<Field> goal);
    void updatePosition(std::shared_ptr<Field> field);
    void updateInitialPosition(std::shared_ptr<Field> field);
    void updateObjective(Objective obj);
    void updateHeading(int heading);
    void updateArrow(bool arrow);
    void updateHaveGold(bool gold);
    void updateHaveSafePathToGold();
    void updateExhausted(bool exhausted);
    void updateShot();
    void updateBlockingWumpi(std::unordered_set<std::shared_ptr<wumpus::model::Field>> blockingWumpi);
    void updateBlockingTraps(std::unordered_set<std::shared_ptr<wumpus::model::Field>> blockingTraps);

    bool isBlockedByTrap();
    bool isBlockedByWumpus();

    void setExited();
    void setDead(const std::shared_ptr<wumpus::model::Field>& diedOn);

private:
    bool registered;



};

} /* namespace model */
} /* namespace wumpus */
