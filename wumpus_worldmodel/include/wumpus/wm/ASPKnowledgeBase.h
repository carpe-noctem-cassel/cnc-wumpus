#pragma once
#include <asp_solver/ASPSolver.h>
#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <vector>
#include <mutex>

namespace wumpus
{
class WumpusWorldModel;
namespace wm
{
class KnowledgeManager;
class ASPKnowledgeBase
{
  public:
    ASPKnowledgeBase(KnowledgeManager *wm);
    virtual ~ASPKnowledgeBase();
    void initializeSolver(::reasoner::ASPSolver* solver);
    std::vector<string> solve(string queryRule);
    int addInformation(std::vector<string>& information, int lifetime = -1);
    bool revoke(int queryId);

  private:
    KnowledgeManager *km;
    ::reasoner::ASPSolver *solver;
    std::vector<std::string> currentActionSequence;
	std::mutex mtx;

};

} /* namespace wm */
} /* namespace wumpus */
