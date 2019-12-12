#include "aspkb/TermManager.h"
#include <reasoner/asp/IncrementalExtensionQuery.h>
#include <reasoner/asp/ReusableExtensionQuery.h>

namespace aspkb
{

TermManager& TermManager::getInstance()
{
    static TermManager instance;
    return instance;
}

/**
 * Creates and saves a Term pointer with id, queryId, externals and default lifetime of -1
 * @return Pointer to new term
 */
::reasoner::asp::Term* TermManager::requestTerm()
{
    auto term = new ::reasoner::asp::Term();
    auto id = solver->generateQueryID();
    term->setLifeTime(-1);
    term->setId(id);
    term->setExternals(std::make_shared<std::map<std::string, bool>>());
    managedTerms.push_back(term);
    return term;
}

void TermManager::initializeSolver(::reasoner::asp::Solver* solver)
{
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
        id = solver->generateQueryID();
        term->setLifeTime(1);
        term->setId(id);
        term->setType(reasoner::asp::ReusableExtension);
        term->setExternals(std::make_shared<std::map<std::string, bool>>());
        for (auto rule : rules) {
            term->addRule(rule);
        }
        this->managedTerms.push_back(term);
        auto query = std::make_shared<reasoner::asp::ReusableExtensionQuery>(id, this->solver, term);
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
    auto id = solver->generateQueryID();
    checkTerm->setId(id);
    checkTerm->setExternals(std::make_shared<std::map<std::string, bool>>());
    this->managedTerms.push_back(checkTerm);
    checkTerm->setLifeTime(1); // TODO is this correct?
    checkTerm->setType(reasoner::asp::QueryType::Extension);
    checkTerm->addProgramSectionParameter("t", std::to_string(horizon));
    // for (const auto& str : checkRules) {
    auto rule = "notMoved(t) :- incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
                std::string("(holds(on(A,B),t)) , incquery") + std::to_string(0) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
                std::string("(holds(on(A,B),0)).");
    checkTerm->addRule(rule);
    rule = ":- not goal(_,_), notMoved(t).";
    checkTerm->addRule(rule);
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
    rule = ":- not unsafeMovesAllowed, movedInDanger(t).";
    checkTerm->addRule(rule);

    rule = " endsOnVisited(t) :- visited(X,Y), incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
           std::string("(holds(on(X,Y)," + std::to_string(horizon) + ")).");
    checkTerm->addRule(rule);

    rule = ":- not goal(_,_), endsOnVisited(t).";
    checkTerm->addRule(rule);
    rule = "wrongHeading(t) :- goalHeading(A), incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
           std::string("(holds(heading(B),t)), A!=B.");
    checkTerm->addRule(rule);

    rule = ":- wrongHeading(t).";
    checkTerm->addRule(rule);
    rule = "notEndsOnGoal(t) :- goal(X,Y), not incquery" + std::to_string(horizon) /* + std::to_string(reasoner::asp::IncrementalExtensionQuery::queryId) */ +
           std::string("(holds(on(X,Y),t)).");

    checkTerm->addRule(rule);
    rule = ":- notEndsOnGoal(t).";

    checkTerm->addRule(rule);
    checkTerm->addRule("randomQueryFact(t).");
    return checkTerm;
}

void TermManager::clear()
{
    this->solver = nullptr;
    this->reusableQueries.clear();
    for (auto it = managedTerms.begin(); it < managedTerms.end(); ++it) {
        delete *it;
        managedTerms.erase(it);
    }
}
}
