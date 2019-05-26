#pragma once
#include "DomainElement.h"
#include <map>
#include <memory>
#include <mutex>
#include <utility> //pair
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
    void handleSilence();
    void handleScream();
    std::shared_ptr<wumpus::model::Agent> getAgentById(int id);
    std::shared_ptr<wumpus::model::Field> getField(int x, int y);
    int getPlaygroundSize();
    int getNumberOfAgents();
    int getNumberOfFields();

private:
    long turnCounter;
    int playgroundSize;
    std::map<int, std::shared_ptr<wumpus::model::Agent>> agents;
    std::map<std::pair<int, int>, std::shared_ptr<wumpus::model::Field>> fields;
    std::mutex agentMtx;
    std::mutex fieldMtx;
    std::mutex turnMtx;
};

} /* namespace model */
} /* namespace wumpus */
