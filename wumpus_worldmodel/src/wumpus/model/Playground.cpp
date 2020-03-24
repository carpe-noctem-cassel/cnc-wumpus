#include "wumpus/model/Playground.h"
#include "wumpus/model/Agent.h"
#include "wumpus/model/Field.h"
#include <nonstd/optional.hpp>
#include <wumpus/WumpusWorldModel.h>

namespace wumpus
{
namespace model
{

std::mutex Playground::agentMtx;
std::mutex Playground::fieldMtx;

Playground::Playground(wumpus::wm::ChangeHandler* ch)
        : DomainElement(ch)
{
    this->playgroundSize = -1;
    this->turnCounter = 0;
}

Playground::~Playground() = default;

void Playground::addAgent(std::shared_ptr<wumpus::model::Agent> agent)
{
    std::lock_guard<std::mutex> lock(this->agentMtx);
    this->agents[agent->id] = agent;
}
void Playground::addAgentForExperiment(std::shared_ptr<wumpus::model::Agent> agent)
{
    this->agentsForExperiment[agent->id] = agent;
}

void Playground::removeAgent(int id)
{
    std::lock_guard<std::mutex> lock(this->agentMtx);
    if (this->agents.find(id) != this->agents.end()) {
        this->agents.erase(id);
    }
}
std::shared_ptr<wumpus::model::Agent> Playground::getAgentById(int id)
{
    std::lock_guard<std::mutex> lock(this->agentMtx);
    auto it = this->agents.find(id);
    if (it != this->agents.end()) {
        return it->second;
    }
    return nullptr;
}

/**
 * Returns agents currently known to the playground
 * @param expectAll If true, the list of agents is only returned if it contains as many elements as agents are expected to be part of the experiment
 * @return
 */
std::shared_ptr<std::map<int, std::shared_ptr<wumpus::model::Agent>>> Playground::getAgents(bool expectAll)
{
    if (expectAll) {

        if (this->agents.size() < wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount()) {
            return nullptr;
        }
    }

    return std::make_shared<std::map<int, std::shared_ptr<wumpus::model::Agent>>>(this->agents);
}

std::shared_ptr<std::map<int, std::shared_ptr<wumpus::model::Agent>>> Playground::getAgentsForExperiment()
{
    if (this->agentsForExperiment.size() < wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount()) {
        return nullptr;
    }
    return std::make_shared<std::map<int, std::shared_ptr<wumpus::model::Agent>>>(this->agentsForExperiment);
}

std::shared_ptr<wumpus::model::Field> Playground::getField(int x, int y)
{
    std::lock_guard<std::mutex> lock(this->fieldMtx);
    auto it = this->fields.find(std::make_pair(x, y));

    if (it != this->fields.end()) {
        return it->second;
    }

    // playground not initialized yet
    auto field = std::make_shared<wumpus::model::Field>(this->ch, x, y);
    this->fields[std::make_pair(x, y)] = field;

    return field;
}

/**
 * Sets field size and creates fields
 */
void Playground::initializePlayground(int size)
{
    std::lock_guard<std::mutex> lock(this->fieldMtx);
    this->playgroundSize = size;
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            auto coordinates = std::make_pair(i, j);
            // check if field already exists
            if (this->fields.find(coordinates) == this->fields.end()) {
                auto field = std::make_shared<wumpus::model::Field>(this->ch, i, j);
                this->fields[coordinates] = field;
            }
        }
    }
    this->ch->handleSetFieldSize(size);
}

/**
 * Convenience method to distribute things to agents.
 * Requires that list of agents and list of things have a defined order.
 * @return
 */
nonstd::optional<int> Playground::getOwnAgentIndex()
{
    std::lock_guard<std::mutex> lock(this->agentMtx);
    auto ids = wumpus::WumpusWorldModel::getInstance()->getAgentIDsForExperiment();
    //    std::cout << "ids: " << ids.size() << std::endl;
    for (int i = 0; i < ids.size(); ++i) {
        auto agent = ids.at(i);
        if (agent == essentials::SystemConfig::getOwnRobotID()) {
            return i;
        }
    }

    return nonstd::nullopt;
}

std::vector<std::shared_ptr<wumpus::model::Field>> Playground::getAdjacentFields(int x, int y)
{
    std::lock_guard<std::mutex> lock(this->fieldMtx);
    std::vector<std::shared_ptr<wumpus::model::Field>> adj;
    auto field = this->fields.find(std::make_pair(x + 1, y));
    if (field != this->fields.end()) {
        adj.push_back(field->second);
    }
    field = this->fields.find(std::make_pair(x - 1, y));
    if (field != this->fields.end()) {
        adj.push_back(field->second);
    }
    field = this->fields.find(std::make_pair(x, y + 1));
    if (field != this->fields.end()) {
        adj.push_back(field->second);
    }
    field = this->fields.find(std::make_pair(x, y - 1));
    if (field != this->fields.end()) {
        adj.push_back(field->second);
    }
    return adj;
}

void Playground::handleSilence()
{
    this->ch->handleSilence();
}

void Playground::handleScream()
{
    this->ch->handleScream();
}

int Playground::getPlaygroundSize()
{
    return this->playgroundSize;
}

int Playground::getNumberOfFields()
{
    return this->playgroundSize * this->playgroundSize;
}

int Playground::getNumberOfAgents()
{
    return this->agents.size();
}

std::vector<std::shared_ptr<wumpus::model::Agent>> Playground::getAgentsWhoShot()
{
    std::vector<std::shared_ptr<wumpus::model::Agent>> ret;
    for(const auto& a : *this->getAgents(false)) {
        if(a.second->shot) {
            ret.push_back(a.second);
        }
    }
    return ret;
}

} /* namespace model */
} /* namespace wumpus */
