#include "aspkb/TermManager.h"
#include <reasoner/asp/IncrementalExtensionQuery.h>
#include <reasoner/asp/ReusableExtensionQuery.h>

namespace aspkb
{
std::mutex TermManager::queryMtx;
TermManager& TermManager::getInstance()
{
    static TermManager instance;
    return instance;
}

std::mutex TermManager::mtx;
/**
 * Creates and saves a Term pointer with id, queryId, externals and default lifetime of -1
 * @return Pointer to new term
 */
::reasoner::asp::Term* TermManager::requestTerm()
{
    auto term = new ::reasoner::asp::Term();
    auto id = solver->getQueryCounter();
    term->setLifeTime(-1);
    term->setId(id);
    term->setQueryId(id);
    term->setExternals(std::make_shared<std::map<std::string, bool>>());
    std::lock_guard<std::mutex> lock(this->mtx);
    managedTerms.push_back(term);
    return term;
}

void TermManager::initializeSolver(::reasoner::asp::Solver* solver)
{
    std::cout << "TERMMANAGER: INIT SOLVER!!!" << std::endl;
    if (this->solver == nullptr) {
        this->solver = solver;
    }
}

::reasoner::asp::Solver* TermManager::getSolver() const
{
    return this->solver;
}

// TODO delete pointers to terms as soon as their associated queries expires
TermManager::~TermManager()
{
    std::cout << "TermManager Destructor" << std::endl;
    for (auto it = managedTerms.begin(); it < managedTerms.end(); ++it) {
        delete *it;
        managedTerms.erase(it);
    }
}

int TermManager::activateReusableExtensionQuery(std::string identifier, const std::vector<std::string>& rules)
{
    int id = -1;
    if (this->reusableQueries.find(identifier) == this->reusableQueries.end()) {
        //        std::cout << "Creating term with identifier " << identifier << std::endl;
        auto term = new reasoner::asp::Term();
        id = solver->getQueryCounter();
        term->setLifeTime(1);
        term->setId(id);
        term->setQueryId(id);
        term->setType(reasoner::asp::ReusableExtension);
        term->setExternals(std::make_shared<std::map<std::string, bool>>());
        for (auto rule : rules) {
            term->addRule(rule);
        }
        std::lock_guard<std::mutex> lock(this->mtx);

        this->managedTerms.push_back(term);
        auto query = std::make_shared<reasoner::asp::ReusableExtensionQuery>(this->solver, term);
        this->solver->registerQuery(query);
        this->reusableQueries.emplace(identifier, query);
    } else {
        //        std::cout << "Found identifier " << identifier << std::endl;
        auto query = this->reusableQueries.at(identifier);
        query->reactivate();
        id = query->getTerm()->getId();
    }
    return id;
}

// TODO belongs somewhere else
::reasoner::asp::Term* TermManager::requestCheckTerm(int horizon)
{
    auto checkTerm = new ::reasoner::asp::Term();
    auto id = solver->getQueryCounter();
    checkTerm->setId(id);
    checkTerm->setQueryId(id);
    checkTerm->setExternals(std::make_shared<std::map<std::string, bool>>());
    std::lock_guard<std::mutex> lock(this->mtx);
    this->managedTerms.push_back(checkTerm);
    checkTerm->setLifeTime(-1); // TODO is this correct?
                                //    checkTerm->setLifeTime(1); // TODO is this correct?
    checkTerm->setType(reasoner::asp::QueryType::ReusableExtension);
    //    checkTerm->setType(reasoner::asp::QueryType::Extension);
    checkTerm->addProgramSectionParameter("t", std::to_string(horizon));
    // for (const auto& str : checkRules) {
    auto rule = "notMoved(t) :- incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
                std::string("(holds(on(A,B),t)) , incquery") + std::to_string(0) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
                std::string("(holds(on(A,B),0)).");
    checkTerm->addRule(rule);
    rule = ":- not goal(_,_), notMoved(t).";
    checkTerm->addRule(rule); // FIXME comment in
    for (int i = 0; i <= horizon; ++i) {
        rule = "movedInDanger(t) :- wumpus(X,Y) , incquery" + std::to_string(i) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
               std::string("(holds(on(X,Y)," + std::to_string(i) + ")).");
        checkTerm->addRule(rule);
    }

    for (int i = 0; i <= horizon; ++i) {
        rule = "movedInDanger(t) :- trap(X,Y) , incquery" + std::to_string(i) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
               std::string("(holds(on(X,Y)," + std::to_string(i) + ")).");
        checkTerm->addRule(rule);
    }
    rule = ":- not unsafeMovesAllowed, not shotAt(_,_) , movedInDanger(t)."; // FIXME check if agent actually shot and possibly revoke
    checkTerm->addRule(rule);

    rule = ":- movedInDanger(t), haveGold(A), me(A)."; // agent should go home safely only!
    checkTerm->addRule(rule);

//    rule = " endsOnVisited(t) :- visited(X,Y), incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
//           std::string("(holds(on(X,Y)," + std::to_string(horizon) + ")).");
//    checkTerm->addRule(rule);
//    rule = ":- not goal(_,_), endsOnVisited(t).";
//    checkTerm->addRule(rule);

    rule = " endsOnExplored(t) :- explored(X,Y), incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
           std::string("(holds(on(X,Y)," + std::to_string(horizon) + ")).");
    checkTerm->addRule(rule); //FIXME evaluate using explored vs visited here
    rule = ":- not goal(_,_), endsOnExplored(t).";

    checkTerm->addRule(rule);
    rule = "wrongHeading(t) :- goalHeading(A), incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
           std::string("(holds(heading(B),t)), A!=B.");
    checkTerm->addRule(rule);

    rule = ":- wrongHeading(t).";
    checkTerm->addRule(rule);
    rule = "notEndsOnGoal(t) :- goal(X,Y), not incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
           std::string("(holds(on(X,Y),t)).");

    //FIXME exploring safe fields close to the (glittering) goal should be more favourable than allowing

    checkTerm->addRule(rule);
    rule = ":- notEndsOnGoal(t).";

    checkTerm->addRule(rule);
    checkTerm->addRule("randomQueryFact(t).");
//    checkTerm->addRule(":- incquery0(holds(on(_,_),0)).");
    return checkTerm;
}

void TermManager::clear()
{
    std::cout << "Start clearing TermManager" << std::endl;
    std::lock_guard<std::mutex> lock(this->mtx);
    this->solver = nullptr;
    this->reusableQueries.clear();
    for (auto it = managedTerms.begin(); it < managedTerms.end(); ++it) {
        delete *it;
        managedTerms.erase(it);
    }
}
}
