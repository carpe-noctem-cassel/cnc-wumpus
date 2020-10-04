#include "wumpus/WumpusWorldModel.h"

#include "wumpus/wm/ChangeHandler.h"
#include "wumpus/wm/PlanningModule.h"
#include <engine/AlicaEngine.h>
#include <engine/IRoleAssignment.h>
//#include <aspkb/Integrator.h>
#include <engine/teammanager/TeamManager.h>

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
        , extractor(nullptr)
        , integrator(nullptr)
        , planningModule(nullptr)
        , playground(nullptr)
        , communication(nullptr)
        , experiment(nullptr)
        , localAgentExited(false)
        , localAgentDied(false)
        , isTimedOut(false)
        , performActionSuccess(false)

{
    auto sc = essentials::SystemConfig::getInstance();
    this->agentName = essentials::SystemConfig::getHostname();
    this->agentCount = (*sc)["WumpusWorldModel"]->get<int>("Agents.number", NULL);
    this->agentIDs = (*sc)["WumpusWorldModel"]->getList<int>("Agents.ids", NULL);
    this->timeoutDurationSeconds = (*sc)["WumpusEval"]->get<int>("TestRun.timeoutDuration", NULL);
    this->spawnRequestHandlerRoleName = (*sc)["WumpusEval"]->get<std::string>("TestRun.spawnRequestHandlerRoleName", NULL);

    if (this->agentIDs.size() != this->agentCount) {
        throw std::runtime_error("WumpusWorldModel.conf: Length of id list has to match defined number of agents");
    }
}

WumpusWorldModel::~WumpusWorldModel()
{
    delete this->communication;
    delete this->changeHandler;
    delete this->planningModule;
    delete this->extractor;
    delete this->integrator;
    delete this->playground;
    delete this->experiment;
}

std::string WumpusWorldModel::getAgentName()
{
    return this->agentName;
}

void WumpusWorldModel::init()
{

    this->extractor = new aspkb::Extractor();
    this->integrator = new aspkb::Integrator();
    this->changeHandler = new wumpus::wm::ChangeHandler(this, integrator);
    this->planningModule = new wumpus::wm::PlanningModule(this, extractor, integrator);
    this->playground = new model::Playground(this->changeHandler);
    this->communication = new wm::Communication(this);
    this->experiment = new eval::Experiment();
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
std::vector<int> WumpusWorldModel::getAgentIDsForExperiment()
{
    return this->agentIDs;
}

bool WumpusWorldModel::localAgentIsSpawnRequestHandler()
{
    auto id = this->getEngine()->getTeamManager()->getLocalAgentID();
    return this->getEngine()->getRoleAssignment()->getRole(id)->getName() == this->spawnRequestHandlerRoleName;
}

void WumpusWorldModel::reset()
{
    std::lock_guard<std::mutex> lock(this->resetMtx); // FIXME
    this->setPerformActionSuccess(false);
    // delete this->communication;
    std::cout << "Resetting wm!" << std::endl;
    delete this->changeHandler; // FIXME only clear state
    std::cout << "Resetting wm: PM!" << std::endl;
    delete this->planningModule;
    delete this->extractor;
    delete this->integrator;
    std::cout << "Resetting wm!: PG" << std::endl;
    delete this->playground;
    this->localAgentExited = false;
    this->localAgentDied = false;
    std::cout << "Resetting SimData!" << std::endl;

    this->wumpusSimData.clearBuffers();

    this->extractor = new aspkb::Extractor();
    this->integrator = new aspkb::Integrator();
    this->changeHandler = new wumpus::wm::ChangeHandler(this, integrator);
    this->planningModule = new wumpus::wm::PlanningModule(this, extractor, integrator);
    std::cout << "Resetting wm!: New Playground" << std::endl;

    this->playground = new model::Playground(this->changeHandler);
    std::cout << "Resetting wm!: Done " << std::endl;

    this->resettedForEncoding.emplace(this->currentEncoding);
    std::cout << "Resetting wm!: resetted for encoding " << this->currentEncoding << std::endl;
    this->isTimedOut = false;
}

void WumpusWorldModel::setIsTimeout()
{
    this->isTimedOut = true;
}

bool WumpusWorldModel::isTimeout()
{
    return this->isTimedOut;
}


void WumpusWorldModel::setPerformActionSuccess(bool success)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    this->performActionSuccess = success;
}
bool WumpusWorldModel::isPerformActionSuccess()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->performActionSuccess;
}

/**
 * FIXME weird structure because of refactoring...
 */
// void WumpusWorldModel::integrateChanges()
//{
//    this->changeHandler->integrator->applyChanges();
//}

} /* namespace wumpus */
