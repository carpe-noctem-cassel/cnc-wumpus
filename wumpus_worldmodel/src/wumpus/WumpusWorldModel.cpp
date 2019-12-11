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
        , experiment(nullptr)
        , localAgentExited(false)
        , localAgentDied(false)

{
    auto sc = essentials::SystemConfig::getInstance();
    this->agentName = essentials::SystemConfig::getHostname();
    this->agentCount = (*sc)["WumpusWorldModel"]->get<int>("Agents.number", NULL);
    this->agentIDs = (*sc)["WumpusWorldModel"]->getList<int>("Agents.ids", NULL);
    if(this->agentIDs.size() != this->agentCount) {
        throw std::runtime_error("WumpusWorldModel.conf: Length of id list has to match defined number of agents");
    }
}

WumpusWorldModel::~WumpusWorldModel()
{
    delete this->communication;
    delete this->changeHandler;
    delete this->planningModule;
    delete this->playground;
    delete this->experiment;
}

std::string WumpusWorldModel::getAgentName()
{
    return this->agentName;
}

void WumpusWorldModel::init()
{

    this->changeHandler = new wumpus::wm::ChangeHandler(this);
    this->planningModule = new wumpus::wm::PlanningModule(this);
    this->playground = new model::Playground(this->changeHandler);
    this->communication = new wm::Communication(this);
    this->experiment = new eval::Experiment();
}

//TODO why is this here?
std::vector<std::pair<std::string, std::string>> WumpusWorldModel::getShotAtFields()
{
    return this->planningModule->shootingTargets;
}

// used in plans to check if all agents have been spawned, only then are actions available
int WumpusWorldModel::getPresetAgentCount()
{
    return this->agentCount;
}

/**
 * Returns pre-defined list of agent ids from config.
 * This is only to make evaluation easier but doesn't make much sense
 * for general usage since there is no validation/consistency checking in the
 * simulator.
 * @return list of agent ids which are going to participate in the experiment
 */
std::vector<int> WumpusWorldModel::getAgentIDsForExperiment() {
    return this->agentIDs;
}

void WumpusWorldModel::reset()
{
    //delete this->communication;
    delete this->changeHandler; //FIXME only clear state
    delete this->planningModule;
    delete this->playground;
    this->localAgentExited = false;
    this->localAgentDied = false;
    this->wumpusSimData.clearBuffers();


    this->changeHandler = new wumpus::wm::ChangeHandler(this);
    this->planningModule = new wumpus::wm::PlanningModule(this);
    this->playground = new model::Playground(this->changeHandler);
}

} /* namespace wumpus */
