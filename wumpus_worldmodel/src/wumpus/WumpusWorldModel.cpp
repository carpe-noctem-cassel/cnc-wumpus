#include "wumpus/WumpusWorldModel.h"

#include "wumpus/wm/ChangeHandler.h"
#include "wumpus/wm/PlanningModule.h"
#include <engine/AlicaEngine.h>

namespace wumpus
{

WumpusWorldModel* WumpusWorldModel::getInstance()
{
    static WumpusWorldModel instance;
    return &instance;
}

WumpusWorldModel::WumpusWorldModel()
        : WorldModel()
        , wumpusSimData(this)
        , changeHandler(nullptr)
        , planningModule(nullptr)
        , playground(nullptr)
        , communication(nullptr)
        , localAgentExited(false)
        , localAgentDied(false)
{
    this->agentName = essentials::SystemConfig::getHostname();
    this->agentCount = (*sc)["WumpusWorldModel"]->get<int>("Agents.number", NULL);
}

WumpusWorldModel::~WumpusWorldModel()
{
    delete this->communication;
    delete this->changeHandler;
    delete this->planningModule;
    delete this->playground;
}

std::string WumpusWorldModel::getAgentName()
{
    return this->agentName;
}

void WumpusWorldModel::init()
{
    auto sc = essentials::SystemConfig::getInstance();


    this->changeHandler = new wumpus::wm::ChangeHandler(this);
    this->planningModule = new wumpus::wm::PlanningModule(this);
    this->playground = new model::Playground(this->changeHandler);
    this->communication = new wm::Communication(this);
}

void WumpusWorldModel::clear() {
    delete this->communication;
    delete this->changeHandler;
    delete this->planningModule;
    delete this->playground;
    this->localAgentExited = false;
    this->localAgentDied = false;
    this->wumpusSimData.clearBuffers();

}

//used in plans to check if all agents have been spawned, only then are actions available
int WumpusWorldModel::getPresetAgentCount() {
    return this->agentCount;

}

} /* namespace wumpus */
