#include "wumpus/wm/KnowledgeManager.h"
#include "wumpus/WumpusWorldModel.h"
#include "wumpus/wm/ASPKnowledgeBase.h"
#include "wumpus/wm/Field.h"
#include <asp_solver/ASPSolver.h>
#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <engine/AlicaEngine.h>

#define KM_DEBUG

namespace wumpus
{
namespace wm
{

KnowledgeManager::KnowledgeManager(WumpusWorldModel *wm)
    : wm(wm)
{
    this->kb = new ASPKnowledgeBase(this);
    this->queryCount = 0;
}

KnowledgeManager::~KnowledgeManager()
{
    delete this->kb;
}

void KnowledgeManager::initializeSolver()
{
    auto solver = this->wm->getEngine()->getSolver<alica::reasoner::ASPSolverWrapper>()->getSolver();
    this->kb->initializeSolver(static_cast<::reasoner::ASPSolver *>(solver));
}

std::vector<WumpusEnums::actions> KnowledgeManager::getNextAction()
{
    auto actions = std::vector<WumpusEnums::actions>();
    stringstream ss;
    ss << "nextAction(wildcard)";
    auto result = this->kb->solve(ss.str());
    for (auto elem : result)
    {
        auto start = elem.find("(");
        auto end = elem.find(")");
        auto tmp = elem.substr(start + 1, end - start -1);

        if (tmp.compare("move") == 0)
        {
        	cout << "MOVE!" << endl;
            actions.push_back(WumpusEnums::move);
        }
        else if (tmp.compare("turnRight") == 0)
        {
        	cout << "RIGHT!" << endl;
            actions.push_back(WumpusEnums::turnRight);
        }
        else if (tmp.compare("turnLeft") == 0)
        {
        	cout << "LEFT!" << endl;
            actions.push_back(WumpusEnums::turnLeft);
        }
        else if (tmp.compare("shoot") == 0)
        {
        	cout << "SHOOT!" << endl;
            actions.push_back(WumpusEnums::actions::shoot);
        }
        else if (tmp.compare("pickupGold") == 0)
        {
        	cout << "PICKUP!" << endl;
            actions.push_back(WumpusEnums::actions::pickUpGold);
        }
        else if (tmp.compare("leave") == 0)
        {
        	cout << "LEAVE!" << endl;
            actions.push_back(WumpusEnums::actions::leave);
        }
    }

    return actions;
}

// TODO make interface
/**
 * check if there is older information associated with this field and remove it
 */
void KnowledgeManager::updateKnowledgeBase(shared_ptr<wumpus::wm::Field> field)
{
    auto it = this->fieldQueryMap.find(field);
    if (it != this->fieldQueryMap.end())
    {
#ifdef KM_DEBUG
        cout << "KM: Trying to revoke" << it->second << endl;
#endif
        this->kb->revoke(it->second);
    }

    auto id = this->kb->addInformation(field->perceptions);
#ifdef KM_DEBUG
    cout << "KM: AddInformation returned " << id << endl;
#endif

    if (id > -1)
    {
        this->fieldQueryMap[field] = id;
    }
}

void KnowledgeManager::updateKnowledgeBase(shared_ptr<wumpus::wm::GlobalInfo> global)
{
    auto it = this->globalQueryMap.find(global);
    if (it != this->globalQueryMap.end())
    {
        this->kb->revoke(it->second);
    }

    //	this->kb->addInformation(global->perceptions);
}

/**
 * this is only intended for setting constants like field size
 */
void KnowledgeManager::updateKnowledgeBase(string info)
{
    auto information = std::vector<string>();
    information.push_back(info);
    this->kb->addInformation(information);
}
void KnowledgeManager::updateKnowledgeBase(shared_ptr<wumpus::wm::WumpusAgent> agent)
{
}

} /* namespace wm */
} /* namespace wumpus */
