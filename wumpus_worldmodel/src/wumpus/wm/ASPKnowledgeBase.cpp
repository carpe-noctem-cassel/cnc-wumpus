#include "wumpus/wm/ASPKnowledgeBase.h"
#include <asp_commons/ASPCommonsTerm.h>
#include <asp_commons/ASPCommonsVariable.h>
#include <asp_commons/AnnotatedValVec.h>
#include <asp_solver/ASPSolver.h>
#include <asp_solver/ASPVariableQuery.h>
#include <asp_solver_wrapper/ASPSolverWrapper.h>

#define ASPKB_DEBUG

namespace wumpus
{
namespace wm
{

ASPKnowledgeBase::ASPKnowledgeBase(KnowledgeManager *km)
    : km(km)
    , solver(nullptr)
{
}

ASPKnowledgeBase::~ASPKnowledgeBase()
{
}

/**
 *TODO would it be better to filter results with a facts query?
 * Solves the current ASP Program of the Knowledge Base according to specified queryRule
 */
std::vector<string> ASPKnowledgeBase::solve(string queryRule)
{

    auto vars = std::vector<std::shared_ptr<reasoner::ASPCommonsVariable>>();
    vars.push_back(make_shared<::reasoner::ASPCommonsVariable>());
    auto terms = std::vector<std::shared_ptr<reasoner::ASPCommonsTerm>>();
    std::vector<reasoner::AnnotatedValVec> results;

    auto ret = std::vector<string>();
    auto t = make_shared<::reasoner::ASPCommonsTerm>(1);
    t->setQueryRule(queryRule);
    t->setType(::reasoner::ASPQueryType::Facts);
    t->setProgramSection("wumpusBackgroundKnowledgeFile");
    lock_guard<mutex> lock(mtx);
    auto queryId = this->solver->getRegisteredQueriesCount();
    // TODO set ids properly
    //============
    t->setId(queryId);
    //============
    t->setQueryId(queryId);
    terms.push_back(t);
    bool sat = this->solver->getSolution(vars, terms, results);
    if (results.size() > 0)
    {

#ifdef ASPKB_DEBUG
        cout << "ASPKB: Found Result!" << endl;
#endif

        for (auto res : results)
        {

            for (int i = 0; i < res.factQueryValues.size(); ++i)
            {
                for (int j = 0; j < res.factQueryValues.at(i).size(); ++j)
                {
                    auto elem = res.factQueryValues.at(i).at(j);
                    cout << "ASPKB: " << i << "," << j << ", " << elem << endl;
                    this->currentActionSequence.push_back(elem);
                }
            }
        }
    }
//    for (auto m : this->solver->getCurrentModels())
//    {
//        for (auto n : m)
//        {
//            cout << "GET CURRENT MODELS: " << n << endl;
//        }
//    }

    // TODO
    return this->currentActionSequence;
}

/**
 * Combines information into a term, wraps it in a query and registers it in the solver
 * @return id of query
 */
int ASPKnowledgeBase::addInformation(std::vector<string> &information, int lifetime)
{
    lock_guard<mutex> lock(mtx);
    shared_ptr<::reasoner::ASPCommonsTerm> term = make_shared<::reasoner::ASPCommonsTerm>(lifetime);
    int queryId = this->solver->getRegisteredQueriesCount();
    //    term->setProgramSection("wumpusBackgroundKnowledgeFile");
    term->setQueryId(queryId);
    stringstream ss;
    for (auto inf : information)
    {
        term->addFact(inf);
    }

    shared_ptr<::reasoner::ASPQuery> query = make_shared<::reasoner::ASPVariableQuery>(this->solver, term);
    bool success = this->solver->registerQuery(query);

#ifdef ASPKB_DEBUG
    cout << "ASPKB: Adding query " << queryId << " was " << (success ? "successful" : "not successful") << endl;
#endif

    if (success)
    {
        return queryId;
    }

    return -1;
}

/**
 * Removes a specified query from solver
 */
bool ASPKnowledgeBase::revoke(int queryId)
{
    lock_guard<mutex> lock(mtx);
    bool success = false;
    auto registered = this->solver->getRegisteredQueries();

    for (auto query : registered)
    {
        int registeredId = query->getTerm()->getQueryId();
        if (registeredId == queryId)
        {
            success = solver->unregisterQuery(query);
        }
    }

#ifdef ASPKB_DEBUG
    cout << "ASPKB: Removing query " << queryId << " was " << (success ? "successful" : "not successful") << endl;
#endif
    return success;
}

/**
 * Helper method. Solver can't be instantiated at construction time
 */
void ASPKnowledgeBase::initializeSolver(::reasoner::ASPSolver *solver)
{
    if (this->solver == nullptr)
    {
#ifdef ASPKB_DEBUG
        cout << "ASPKB: Initialize Solver." << endl;
#endif
        this->solver = solver;
    }
}

} /* namespace wm */
} /* namespace wumpus */
