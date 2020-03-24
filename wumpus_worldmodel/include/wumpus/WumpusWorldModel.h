#pragma once

#include "model/Playground.h"
#include "wm/Communication.h"
#include "wm/WumpusSimData.h"
#include <SystemConfig.h>
#include <essentials/EventTrigger.h>
#include <supplementary/InformationElement.h>
#include <supplementary/WorldModel.h>
#include <wumpus/wm/PlanningModule.h>
#include <eval/Experiment.h>
#include <string>

namespace alica
{
    class AlicaEngine;
    class AlicaClock;
}
namespace essentials
{
class SystemConfig;
}
namespace eval {
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

    // Public Data Access Classes
    wm::WumpusSimData wumpusSimData;
    model::Playground* playground;
    wumpus::wm::ChangeHandler* changeHandler;
    wumpus::wm::PlanningModule* planningModule;
    wm::Communication* communication;
    eval::Experiment* experiment;
    void reset();
    void integrateChanges();
    std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> getShotAtFields(); //TODO move to playground(?) and fix types (might be Fields?)
    std::vector<int> getAgentIDsForExperiment(); //TODO move?
    bool localAgentIsSpawnRequestHandler();

    void setIsTimeout();
    bool isTimeout();

    void sendAgentPerception(wumpus_msgs::AgentPerception& msg);

    int getPresetAgentCount();
    bool localAgentExited;
    bool localAgentDied;
    int timeoutDurationSeconds;
    std::string spawnRequestHandlerRoleName;
    //maybe rename - remember encodings for which a reset has already been performed
    std::set<std::string> resettedForEncoding;
    std::string currentEncoding; //TODO duplicate field
    std::mutex resetMtx;

private:
    WumpusWorldModel(); /**< Private Singleton Constructor */
    std::string agentName;
    int agentCount;
    std::vector<int> agentIDs;
    bool isTimedOut;
};

} /* namespace wumpus */
