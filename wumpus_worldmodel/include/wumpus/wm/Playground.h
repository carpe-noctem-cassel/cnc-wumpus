#pragma once
#include "wumpus/wm/WumpusAgent.h"
#include <memory>
#include <map>
#include <mutex>
#include <utility>
namespace wumpus
{
class WumpusWorldModel;
namespace wm
{
    class WumpusAgent;
    class Playground
    {
      public:
        Playground(WumpusWorldModel *wm);
        virtual ~Playground();
        void initializePlayground(int playgroundSize);
        void addAgent(std::shared_ptr<wumpus::wm::WumpusAgent> agent);
        std::shared_ptr<wumpus::wm::WumpusAgent> getAgentById(int id);
        std::shared_ptr<wumpus::wm::Field> getField(Coordinates coords);
        int getPlaygroundSize();
        int getNumberOfFields();


      private:
        wumpus::WumpusWorldModel *wm;
        int playgroundSize;
        std::map<int, std::shared_ptr<wumpus::wm::WumpusAgent>> agents;
        std::map<std::pair<int,int>, std::shared_ptr<wumpus::wm::Field>> fields;
        std::mutex agentMtx;
        std::mutex fieldMtx;
    };

} /* namespace wm */
} /* namespace wumpus */
