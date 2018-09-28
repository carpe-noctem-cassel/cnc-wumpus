#pragma once
#include "wumpus/wm/WumpusDefinitions.h"
#include <WumpusEnums.h>
#include <map>
#include <vector>
#include <memory>

namespace wumpus
{
class WumpusWorldModel;
namespace wm
{
class Field;
class WumpusAgent;
class ASPKnowledgeBase;

class KnowledgeManager
{
  public:
    KnowledgeManager(WumpusWorldModel *wm);
    virtual ~KnowledgeManager();
    void initializeSolver();
    std::vector<WumpusEnums::actions> getNextAction();
    void updateKnowledgeBase(std::shared_ptr<wumpus::wm::Field> field);
    void updateKnowledgeBase(std::shared_ptr<wumpus::wm::GlobalInfo> global);
    void updateKnowledgeBase(std::shared_ptr<wumpus::wm::WumpusAgent> agent);
    void updateKnowledgeBase(std::string info);

  private:
    ASPKnowledgeBase *kb;
    WumpusWorldModel *wm;
    int64_t queryCount;

    std::map<std::shared_ptr<wumpus::wm::Field>, int> fieldQueryMap;
    std::map<std::shared_ptr<wumpus::wm::GlobalInfo>, int> globalQueryMap;
    std::map<std::shared_ptr<wumpus::wm::WumpusAgent>, int> agentQueryMap;
};

} /* namespace wm */
} /* namespace wumpus */
