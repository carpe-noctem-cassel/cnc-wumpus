#pragma once
#include "DomainElement.h"
#include <map>
#include <memory>
#include <mutex>
#include <utility> //pair
#include <set>
#include <nonstd/optional.hpp>
#include <wumpus/model/Field.h>

namespace wumpus
{
class WumpusWorldModel;
namespace model
{
class Agent;
class Field;
class Playground : public wumpus::model::DomainElement
{
public:
    Playground(wumpus::wm::ChangeHandler* ch);
    virtual ~Playground();
    void initializePlayground(int playgroundSize);
    void addAgent(std::shared_ptr<wumpus::model::Agent> agent);
    void addAgentForExperiment(std::shared_ptr<wumpus::model::Agent> agent);
    void removeAgent(int id);
    void handleSilence();
    void handleScream();
    void updateWumpusBlocksMoves(bool blocks);
    std::shared_ptr<wumpus::model::Agent> getAgentById(int id);
    std::shared_ptr<std::map<int,std::shared_ptr<wumpus::model::Agent>>> getAgents(bool expectAll);
    std::shared_ptr<std::map<int,std::shared_ptr<wumpus::model::Agent>>> getAgentsForExperiment();
    std::shared_ptr<wumpus::model::Field> getField(int x, int y);
    std::vector<std::shared_ptr<wumpus::model::Field>> getAdjacentFields(int x, int y);
    int getPlaygroundSize();
    nonstd::optional<int> getOwnAgentIndex();
    int getNumberOfAgents();
    int getNumberOfFields();
    std::vector<std::shared_ptr<wumpus::model::Agent>> getAgentsWhoShot();

    static std::map<int, std::shared_ptr<wumpus::model::Agent>> agentsForExperiment;


    std::shared_ptr<std::map<int,std::unordered_set<std::shared_ptr<wumpus::model::Field>>>> getFieldsShotAtByAgentIds();
    void addShootingTarget(const int id, const std::pair<std::string, std::string>& shotAt);

    //this is a convenience flag and set to true when a field's glitter flag is set
    bool goldFieldKnown;

    bool wumpusBlocksSafeMoves; //TODO maybe move into agent
private:
    long turnCounter;
    int playgroundSize;
    //currently active agents
    std::map<int, std::shared_ptr<wumpus::model::Agent>> agents;
    //all agents who ever participated in the experiment
    std::map<std::pair<int, int>, std::shared_ptr<wumpus::model::Field>> fields;
    std::shared_ptr<std::map<int, std::unordered_set<std::shared_ptr<wumpus::model::Field>>>> shootingTargets;
    static std::mutex agentMtx;
    static std::mutex fieldMtx;
    std::mutex shotAtMtx;
};

} /* namespace model */
} /* namespace wumpus */
