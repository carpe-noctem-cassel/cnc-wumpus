#pragma once

#include "wm/WumpusSimData.h"
#include "wm/Communication.h"
#include <SystemConfig.h>
#include <supplementary/EventTrigger.h>
#include <supplementary/WorldModel.h>
#include <supplementary/InformationElement.h>
namespace supplementary
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
namespace wm {
class WumpusSimData;
class Communication;
}

class WumpusWorldModel : public supplementary::WorldModel
{
  public:
    static WumpusWorldModel *getInstance(); /**< Singleton Getter */

    virtual ~WumpusWorldModel();
    void init();
    std::string getAgentName();

    // Public Data Access Classes
    wm::WumpusSimData wumpusSimData;
    wm::Communication* communication;

  private:
    WumpusWorldModel(); /**< Private Singleton Constructor */
    std::string agentName;
};

} /* namespace wumpus */

