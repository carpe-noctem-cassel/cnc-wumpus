#pragma once

#include "model/Playground.h"
#include "wm/Communication.h"
#include "wm/WumpusSimData.h"
#include <SystemConfig.h>
#include <essentials/EventTrigger.h>
#include <supplementary/InformationElement.h>
#include <supplementary/WorldModel.h>
#include <wumpus/wm/PlanningModule.h>

namespace essentials
{
class SystemConfig;
}

namespace alica
{
class AlicaEngine;
class AlicaClock;
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
    int getPresetAgentCount();

    // Public Data Access Classes
    wm::WumpusSimData wumpusSimData;
    model::Playground* playground;
    wumpus::wm::ChangeHandler* changeHandler;
    wumpus::wm::PlanningModule* planningModule;
    wm::Communication* communication;

    int agentCount;

    //TODO remove, used to make evaluation easier
    void clear();
    bool localAgentExited;
    bool localAgentDied;

private:
    WumpusWorldModel(); /**< Private Singleton Constructor */
    std::string agentName;
};

} /* namespace wumpus */
