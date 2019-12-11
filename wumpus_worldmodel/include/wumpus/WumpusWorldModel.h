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
    std::vector<std::pair<std::string, std::string>> getShotAtFields();
    std::vector<int> getAgentIDsForExperiment(); //TODO move?
    int getPresetAgentCount();
    bool localAgentExited;
    bool localAgentDied;


private:
    WumpusWorldModel(); /**< Private Singleton Constructor */
    std::string agentName;
    int agentCount;
    std::vector<int> agentIDs;
};

} /* namespace wumpus */
