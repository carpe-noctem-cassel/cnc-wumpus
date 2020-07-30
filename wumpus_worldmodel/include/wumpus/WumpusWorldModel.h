#pragma once

#include "model/Playground.h"
#include "wm/Communication.h"
#include "wm/WumpusSimData.h"
#include <SystemConfig.h>
#include <aspkb/Integrator.h>
#include <essentials/EventTrigger.h>
#include <eval/Experiment.h>
#include <string>
#include <supplementary/InformationElement.h>
#include <supplementary/WorldModel.h>
#include <wumpus/wm/PlanningModule.h>

namespace alica
{
class AlicaEngine;
class AlicaClock;
}
namespace essentials
{
class SystemConfig;
}
namespace eval
{
class Experiment;
}
namespace wumpus
{
namespace model
{
class Agent;
class Playground;
}
namespace wm
{

class WumpusSimData;
class Communication;
class PlanningModule;
class ChangeHandler;
}

class WumpusWorldModel : public supplementary::WorldModel
{
public:
    static WumpusWorldModel* getInstance(); /**< Singleton Getter */

    virtual ~WumpusWorldModel();
    void init();

    std::string getAgentName();

    bool isPerformActionSuccess();
    void setPerformActionSuccess(bool success);

    // Public Data Access Classes
    wm::WumpusSimData wumpusSimData;
    model::Playground* playground;
    wumpus::wm::ChangeHandler* changeHandler;
    wumpus::wm::PlanningModule* planningModule;
    wm::Communication* communication;
    eval::Experiment* experiment;
    void reset();
    //    void integrateChanges();

    std::vector<int> getAgentIDsForExperiment(); // TODO move?
    bool localAgentIsSpawnRequestHandler();

    void setIsTimeout();
    bool isTimeout();

    void sendAgentPerception(wumpus_msgs::AgentPerception& msg);

    int getPresetAgentCount();
    bool localAgentExited;
    bool localAgentDied;
    int timeoutDurationSeconds;
    std::string spawnRequestHandlerRoleName;
    // maybe rename - remember encodings for which a reset has already been performed
    std::set<std::string> resettedForEncoding;
    std::string currentEncoding; // TODO duplicate field
    std::mutex resetMtx;

    static wumpus::model::Objective objectiveFromString(const std::string& objectiveString)
    {
        if (objectiveString == "goHome") {
            return wumpus::model::Objective::GO_HOME;
        } else if (objectiveString == "moveToGoldField")
            return wumpus::model::Objective::MOVE_TO_GOLD_FIELD;
        else if (objectiveString == "huntWumpus") {
            return wumpus::model::Objective::HUNT_WUMPUS;
        } else if (objectiveString == "fetchOtherAgent") {
            return wumpus::model::Objective::FETCH_OTHER_AGENT;
        } else if (objectiveString == "shoot") {
            return wumpus::model::Objective::SHOOT;
        } else if (objectiveString == "idle") {
            return wumpus::model::Objective::IDLE;
        } else if (objectiveString == "leave") {
            return wumpus::model::Objective::LEAVE;
        } else if (objectiveString == "collectGold") {
            return wumpus::model::Objective::COLLECT_GOLD;
        } else if (objectiveString == "explore") {
            return wumpus::model::Objective::EXPLORE;
        }
        std::cout << "Unknown objective string!" << std::endl;
        throw std::exception();
        return wumpus::model::Objective::UNDEFINED;
    }

    aspkb::Extractor* extractor;
private:
    WumpusWorldModel(); /**< Private Singleton Constructor */
    std::string agentName;
    int agentCount;
    std::vector<int> agentIDs;
    bool isTimedOut;
    aspkb::Integrator* integrator;
    bool performActionSuccess;
    std::mutex mtx;
};

} /* namespace wumpus */
