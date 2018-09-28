#pragma once

#include "wm/Communication.h"
#include "wm/Playground.h"
#include "wm/KnowledgeManager.h"
#include "wm/WumpusSimData.h"
#include <SystemConfig.h>
#include <supplementary/EventTrigger.h>
#include <supplementary/InformationElement.h>
#include <supplementary/WorldModel.h>
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
namespace wm
{
class WumpusAgent;
class Playground;
class WumpusSimData;
class KnowledgeManager;
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
    wm::Playground playground;
    wm::KnowledgeManager knowledgeManager;
    wm::Communication *communication;


  private:
    WumpusWorldModel(); /**< Private Singleton Constructor */
    std::string agentName;
};

} /* namespace wumpus */
