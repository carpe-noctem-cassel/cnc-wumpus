#include "aspkb/Extractor.h"
#include "aspkb/TermManager.h"
#include <FileSystem.h>
#include <engine/AlicaClock.h>
#include <reasoner/asp/AnnotatedValVec.h>
#include <reasoner/asp/ExtensionQuery.h>
#include <reasoner/asp/IncrementalExtensionQuery.h>
#include <reasoner/asp/Term.h>
#include <reasoner/asp/Variable.h>
#include <wumpus/WumpusWorldModel.h>

namespace aspkb
{
std::mutex Extractor::mtx;
std::mutex Extractor::incrementalMtx;
std::mutex Extractor::reusableMtx;
bool Extractor::wroteHeaderReusable = false;
bool Extractor::wroteHeaderIncremental = false;

Extractor::Extractor()
{
    this->solver = TermManager::getInstance().getSolver();
    this->baseRegistered = false;
    // TODO only for getSolution stats
    this->resultsDirectory = (*essentials::SystemConfig::getInstance())["WumpusEval"]->get<std::string>("TestRun.resultsDirectory", NULL);
    this->filenameIncremental = "getSolutionStatsIncremental.csv";
    this->filenameReusable = "getSolutionStatsReusable.csv";
}

Extractor::~Extractor()
{
    reasoner::asp::IncrementalExtensionQuery::clear();
}

std::vector<std::string> Extractor::solveWithIncrementalExtensionQuery(std::vector<std::string> inquiryPredicates, const std::vector<std::string>& baseRules,
        const std::vector<std::string>& stepRules, const std::vector<std::string>& checkRules, int maxHorizon)
{

    std::vector<std::string> ret;
    auto vars = std::vector<reasoner::asp::Variable*>();
    auto tmpVar = std::make_shared<::reasoner::asp::Variable>();
    vars.push_back(tmpVar.get());
    auto terms = std::vector<reasoner::asp::Term*>();
    std::vector<reasoner::asp::AnnotatedValVec*> results;
    bool sat = false;
    alica::AlicaTime timeElapsed = alica::AlicaTime::zero();
    int horizon = 1;

    // add terms to terms passed in getSolution
    int attempts = 0;

    while (!sat && horizon <= maxHorizon) {
        std::lock_guard<std::mutex> lock(TermManager::queryMtx);
        auto registeredQueries = this->solver->getRegisteredQueries(); // there are new queries added in each iteration
        attempts++;
        terms.clear();
        if (!baseRegistered) { // fixme register in query by hand. make removable or not?
            auto baseTerm = TermManager::getInstance().requestTerm();
            baseTerm->setLifeTime(-1);
            baseTerm->setType(reasoner::asp::QueryType::IncrementalExtension);
            for (const auto& str : baseRules) {
                baseTerm->addRule(str);
            }
            terms.push_back(baseTerm);
            baseRegistered = true;
        }

        if (reasoner::asp::IncrementalExtensionQuery::isPresent(horizon)) {
            reasoner::asp::IncrementalExtensionQuery::activate(horizon);

        } else {

            auto stepTerm = TermManager::getInstance().requestTerm();
            stepTerm->setLifeTime(-1);
            stepTerm->setType(reasoner::asp::QueryType::IncrementalExtension);
            stepTerm->addProgramSectionParameter("t", std::to_string(horizon));
            for (const auto& str : stepRules) {
                stepTerm->addRule(str);
            }
            // manually added rules are not expanded properly :( FIXME
            stepTerm->addRule("{occurs(A,t-1) : moveAction(A)} = 1.");

            terms.push_back(stepTerm);
        }

        // experimenting with check term
        // todo make reusable
        //        auto checkTerm = TermManager::getInstance().requestCheckTerm(horizon);
        //        terms.push_back(checkTerm);

        //        if(horizon > 1) { //TODO lifetime? whose responsibility is it to reactivate queries?
        //            this->checkQueries.at(horizon-1)->removeExternal();
        //        }

        //         if(this->checkQueries.find(horizon) != this->checkQueries.end()) {
        //        auto checkTerm = TermManager::getInstance().requestCheckTerm(horizon);
        //        terms.push_back(checkTerm);

        // FIXME this is not working because of the order of grounding...
        if (horizon > 1) {
            for (const auto& query : registeredQueries) {
                if (query->getTerm()->getQueryId() ==
                        this->checkTerms.at(horizon - 1)->getQueryId()) { // TODO lifetime? whose responsibility is it to reactivate queries?
                    //                this->checkQueries.at(horizon - 1)->removeExternal();
                    query->removeExternal();
                    break;
                }
            }
        }

        if (this->checkTerms.find(horizon) != this->checkTerms.end()) {
            for (const auto& query : registeredQueries) {
                if (query->getTerm()->getQueryId() == this->checkTerms.at(horizon)->getQueryId()) {
                    std::dynamic_pointer_cast<::reasoner::asp::ReusableExtensionQuery>(query)->reactivate();
                    break;
                }
            }
        } else {
            auto checkTerm = TermManager::getInstance().requestCheckTerm(horizon);
            terms.push_back(checkTerm);
            //            auto checkQuery = std::make_shared<reasoner::asp::ReusableExtensionQuery>(this->solver, checkTerm);
            //            this->solver->registerQuery(checkQuery);
            //            this->checkQueries.emplace(horizon, checkQuery);
            this->checkTerms.emplace(horizon, checkTerm);
        }

        // Create Term belonging to a FilterQuery to be part of the terms in the getSolution call

        for (auto inquiry : inquiryPredicates) {
            auto inquiryTerm = TermManager::getInstance().requestTerm();
            for (int j = 0; j <= horizon; ++j) {
                inquiryTerm->setLifeTime(1);
                inquiryTerm->setType(::reasoner::asp::QueryType::Filter);
                // wrap query rule to match extension query
                auto wrappedQueryRule = std::string("incquery") + std::to_string(j) + "(" + inquiry + ")";
                //                std::cout << "adding inquiry term" << std::endl;
                inquiryTerm->setQueryRule(wrappedQueryRule);
                //                std::cout << "inquiry - 2 " << std::endl;
                terms.push_back(inquiryTerm);
            }
        }

        // add terms to terms passed in getSolution
        auto timeBefore = wumpus::WumpusWorldModel::getInstance()->getEngine()->getAlicaClock()->now();
        sat = this->solver->getSolution(vars, terms, results);
        auto solveTime = wumpus::WumpusWorldModel::getInstance()->getEngine()->getAlicaClock()->now() - timeBefore;
        timeElapsed = timeElapsed + solveTime;

        std::cout << "PROBLEM IS " << (sat ? "SATISFIABLE" : "NOT SATISFIABLE") << ", " << std::endl; // inquiryTerm->getQueryRule() << std::endl;
        ++horizon;
    }

    this->writeGetSolutionStatsIncremental(horizon, timeElapsed);

    // FIXME loop in reverse
    std::lock_guard<std::mutex> lock(TermManager::queryMtx);
    auto registeredQueries = this->solver->getRegisteredQueries(); // there are new queries added in each iteration
    for (const auto& term : this->checkTerms) {
        for (const auto& query : registeredQueries) {
            if (query->getTerm()->getQueryId() == term.second->getQueryId()) {
                query->removeExternal();
                break;
            }
        }
    }

    reasoner::asp::IncrementalExtensionQuery::cleanUp();

    // new run should start without active step queries
    //        if(sat) {
    //            for(auto query : this->checkQueries) {
    //                query.second->removeExternal();
    //            }
    //        }

    // TODO something is buggy here (way too many result entries)

    for (auto res : results) {
        for (auto& factQueryValue : res->factQueryValues) {
            for (const auto& elem : factQueryValue) {
                if (std::find(ret.begin(), ret.end(), elem) == ret.end()) {
                    //                        std::cout << "ADDING ELEMENT: " << elem << std::endl;
                    ret.push_back(elem);
                    //                    std::cout << "ELEM: " << elem << std::endl;
                }
            }
        }
        delete res;
    }
    return ret;
}
/*
 * TODO what to do about reusable query (lifetime etc.)
 * TODO time measurements
 */
std::vector<std::string> Extractor::extractReusableTemporaryQueryResult(
        const std::vector<std::string>& inquiryPredicates, const std::string& queryIdentifier, const std::vector<std::string>& additionalRules)
{
    std::vector<std::string> ret;
    auto vars = std::vector<reasoner::asp::Variable*>();
    auto tmpVar = std::make_shared<::reasoner::asp::Variable>();
    vars.push_back(tmpVar.get());
    auto terms = std::vector<reasoner::asp::Term*>();
    std::vector<reasoner::asp::AnnotatedValVec*> results;
    alica::AlicaTime timeElapsed = alica::AlicaTime::zero();

    int id = TermManager::getInstance().activateReusableExtensionQuery(queryIdentifier, additionalRules);

    // TODO make reusable as well
    for (auto inquiry : inquiryPredicates) {
        auto inquiryTerm = TermManager::getInstance().requestTerm();
        inquiryTerm->setLifeTime(1);
        inquiryTerm->setType(::reasoner::asp::QueryType::Filter);
        // wrap query rule to match extension query
        auto wrappedQueryRule = std::string("query") + std::to_string(id) + "(" + inquiry + ")";
        inquiryTerm->setQueryRule(wrappedQueryRule);
        terms.push_back(inquiryTerm);
    }

    auto timeBefore = wumpus::WumpusWorldModel::getInstance()->getEngine()->getAlicaClock()->now();
    auto sat = this->solver->getSolution(vars, terms, results);
    auto solveTime = wumpus::WumpusWorldModel::getInstance()->getEngine()->getAlicaClock()->now() - timeBefore;
    timeElapsed = timeElapsed + solveTime;
    this->writeGetSolutionStatsReusable(queryIdentifier,timeElapsed);
    std::cout << "PROBLEM IS " << (sat ? "SATISFIABLE" : "NOT SATISFIABLE") << ", " << std::endl; // inquiryTerm->getQueryRule() << std::endl;

    for (auto res : results) {
        for (auto& varQueryValue : res->variableQueryValues) {
            for (const auto& elem : varQueryValue) {
                ret.push_back(elem);
            }
        }
        delete res;
    }

    TermManager::getInstance().deactivateReusableExtensionQuery(queryIdentifier);

    return ret;
}

void Extractor::writeGetSolutionStatsIncremental(int horizon, alica::AlicaTime timeElapsed)
{
    std::lock_guard<std::mutex> lock(Extractor::incrementalMtx);
    std::ofstream fileWriter;
    std::string separator = ";";
    if (!aspkb::Extractor::wroteHeaderIncremental) {
        aspkb::Extractor::wroteHeaderIncremental = true;
        this->writeHeader(true);
    }

    auto folder = essentials::FileSystem::combinePaths(getenv("HOME"), this->resultsDirectory);
    fileWriter.open(essentials::FileSystem::combinePaths(folder, this->filenameIncremental), std::ios_base::app);
    fileWriter << std::to_string(horizon);
    fileWriter << separator;
    fileWriter << timeElapsed.inMilliseconds();
    fileWriter << std::endl;
    fileWriter.close();
}

void Extractor::writeGetSolutionStatsReusable(const std::string& queryIdentifier, alica::AlicaTime timeElapsed)
{
    std::lock_guard<std::mutex> lock(Extractor::reusableMtx);
    std::ofstream fileWriter;
    std::string separator = ";";

    if (!aspkb::Extractor::wroteHeaderReusable) {
        aspkb::Extractor::wroteHeaderReusable = true;
        this->writeHeader(false);
    }

    auto folder = essentials::FileSystem::combinePaths(getenv("HOME"), this->resultsDirectory);
    fileWriter.open(essentials::FileSystem::combinePaths(folder, this->filenameReusable), std::ios_base::app);

    fileWriter << queryIdentifier;
    fileWriter << separator;
    fileWriter << timeElapsed.inMilliseconds();
    fileWriter << std::endl;
    fileWriter.close();
}

inline void Extractor::writeHeader(bool incremental)
{
    std::ofstream fileWriter;
    std::string separator = ";";
    std::string fileName;
    if (incremental) {
        fileName = this->filenameIncremental;
    } else {
        fileName = this->filenameReusable;
    }

    auto folder = essentials::FileSystem::combinePaths(getenv("HOME"), this->resultsDirectory);
    fileWriter.open(essentials::FileSystem::combinePaths(folder, fileName), std::ios_base::app);

    fileWriter << (incremental ? "MaxHorizon" : "QueryIdentifier");
    fileWriter << separator;
    fileWriter << "TimeElapsed";
    fileWriter << std::endl;
    fileWriter.close();

}

} /* namespace aspkb */