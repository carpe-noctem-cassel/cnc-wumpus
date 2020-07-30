#pragma once

#include <engine/AlicaEngine.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <wumpus/model/Objective.h>
namespace aspkb
{
class Integrator;
}
namespace wumpus
{
class WumpusWorldModel;
namespace model
{
class Agent;
class Field;
class Playground;
}
namespace wm
{
class PlanningModule;
class ChangeHandler
{

public:
    ChangeHandler(wumpus::WumpusWorldModel* wm, aspkb::Integrator* integrator);
    virtual ~ChangeHandler();
    void registerNewAgent(int id, bool me);
    void unregisterAgent(int id);
    void handleChangedPosition(int id, std::shared_ptr<wumpus::model::Field> newPosition);
    void handleChangedHeading(int newHeading);
    void handleChangedArrow(int agentId, bool arrow);
    void handleChangedStench(std::shared_ptr<wumpus::model::Field> field);
    void handleChangedMoveGoal(int id, std::shared_ptr<wumpus::model::Field> goal);
    void handleChangedVisited(std::shared_ptr<wumpus::model::Field> field, int id, bool truthValue);
    void handleChangedShotAt(int id, std::shared_ptr<wumpus::model::Field> field);
    void handleChangedExplored(std::shared_ptr<wumpus::model::Field> field);
    void handleTurn(long turn);
    void handleSetInitialPosition(std::shared_ptr<wumpus::model::Field> newPosition);
    void handleSetGold(int agentId);
    void handleSetDrafty(std::shared_ptr<wumpus::model::Field> field);
    void handleSetGlitter(std::shared_ptr<wumpus::model::Field> field);
    void handleSetFieldSize(int fieldSize);
    void handleGoalReached(int id);
    void handleChangedExhausted(int id, bool exhausted);
    void handleChangedObjective(int id, wumpus::model::Objective objective);
    void handleSetDiedOn(std::shared_ptr<wumpus::model::Field> field);
    void handleScream();
    void handleSilence();

    aspkb::Integrator* integrator;

    // allow pm to integrate its results as externals
    friend PlanningModule;

    void handleChangedShot(int agentId);

    void handleChangedSafeGoldPath(int i);

    void handleChangedWumpusBlocksMoves(bool blocks);

    void handleChangedBlockedByWumpus(const std::shared_ptr<wumpus::model::Field>& field, int id, bool truthValue);

    void handleChangedBlockedByTrap(const std::shared_ptr<wumpus::model::Field>& field, int id, bool truthValue);

    void handleChangedPossibleNext(const std::shared_ptr<model::Field>& sharedPtr, int id, bool truthValue);

private:
    wumpus::WumpusWorldModel* wm;

    void handleShotAtFields() const;
};

} /*namespace wm */
} /*namespace wumpus */
