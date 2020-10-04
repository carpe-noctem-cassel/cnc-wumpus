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

    int getPresetAgentCount();
    bool localAgentExited;
    bool localAgentDied;
    int timeoutDurationSeconds;
    std::string spawnRequestHandlerRoleName;
    // maybe rename - remember encodings for which a reset has already been performed
    std::set<std::string> resettedForEncoding;
    std::string currentEncoding; // TODO duplicate field
    std::mutex resetMtx;

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
