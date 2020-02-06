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
    ChangeHandler(wumpus::WumpusWorldModel* wm);
    virtual ~ChangeHandler();
    void registerNewAgent(int id, bool me);
    void handleChangedPosition(std::shared_ptr<wumpus::model::Field> newPosition);
    void handleChangedHeading(int newHeading);
    void handleChangedArrow(int agentId,bool arrow);
    void handleChangedStench(std::shared_ptr<wumpus::model::Field> field);
    void handleChangedMoveGoal(int id, std::shared_ptr<wumpus::model::Field> goal);
    void handleChangedVisited(std::shared_ptr<wumpus::model::Field> field);
    void handleChangedShotAt(std::shared_ptr<wumpus::model::Field> field);
    void handleTurn(long turn);
    void handleSetInitialPosition(std::shared_ptr<wumpus::model::Field> newPosition);
    void handleSetGold(int agentId);
    void handleSetDrafty(std::shared_ptr<wumpus::model::Field> field);
    void handleSetGlitter(std::shared_ptr<wumpus::model::Field> field);
    void handleSetFieldSize(int fieldSize);
    void handleGoalReached(int id);
    void handleChangedObjective(int id, wumpus::model::Objective objective);
    void handleScream();
    void handleSilence();
    void clearBlacklist();
    bool getIsIntegrating();



    //allow pm to integrate its results as externals
    friend PlanningModule;

private:
    wumpus::WumpusWorldModel* wm;
    aspkb::Integrator* integrator;

    std::mutex mtx;
};

} /*namespace wm */
} /*namespace wumpus */
