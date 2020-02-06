#include "aspkb/Extractor.h"
#include "aspkb/TermManager.h"
#include <reasoner/asp/AnnotatedValVec.h>
#include <reasoner/asp/ExtensionQuery.h>
#include <reasoner/asp/IncrementalExtensionQuery.h>
#include <reasoner/asp/Term.h>
#include <reasoner/asp/Variable.h>

namespace aspkb
{

Extractor::Extractor()
{
    std::cout << "Creating Extractor" << std::endl;
    this->solver = TermManager::getInstance().getSolver();
    this->baseRegistered = false;
    std::cout << "Created Ext" << std::endl;
}

Extractor::~Extractor() {
    std::cout << "Deleting Extractor" << std::endl;
    reasoner::asp::IncrementalExtensionQuery::clear();
    std::cout << "Deleted Extractor" << std::endl;
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

//    std::lock_guard<std::mutex> lock(mtx);
    int horizon = 1;

    std::cout << "Constructing base term" << std::endl;

    auto baseTerm = TermManager::getInstance().requestTerm();
    baseTerm->setLifeTime(-1);
    baseTerm->setType(reasoner::asp::QueryType::IncrementalExtension);
    for (const auto& str : baseRules) {
        baseTerm->addRule(str);
    }

    // add terms to terms passed in getSolution
    int attempts = 0;
    while (!sat && horizon <= maxHorizon) {
        attempts++;
        terms.clear();
        if (!baseRegistered) { // fixme register in query by hand. make removable or not?
            terms.push_back(baseTerm);
            baseRegistered = true;
        }

        if (reasoner::asp::IncrementalExtensionQuery::isPresent(horizon)) {
            std::cout << "Found query for horizon " << horizon << "- activate" << std::endl;
            reasoner::asp::IncrementalExtensionQuery::activate(horizon);
            std::cout << "Done activating" << std::endl;

        } else {
            std::cout << "new step term" << std::endl;
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
            std::cout << "added step term" << std::endl;
        }

        // experimenting with check term
        //todo make reusable
        auto checkTerm = TermManager::getInstance().requestCheckTerm(horizon);
        terms.push_back(checkTerm);

//        if(horizon > 1) { //TODO lifetime? whose responsibility is it to reactivate queries?
//            this->checkQueries.at(horizon-1)->removeExternal();
//        }

//         if(this->checkQueries.find(horizon) != this->checkQueries.end()) {
//            std::cout << "reactivating with id " << horizon << std::endl;
//            this->checkQueries.at(horizon)->reactivate();
//        } else {
//            std::cout << "registering check term for horizon" << horizon << std::endl;
//            auto checkTerm = TermManager::getInstance().requestCheckTerm(horizon);
//            auto checkQuery = std::make_shared<reasoner::asp::ReusableExtensionQuery>(this->solver, checkTerm);
//            this->solver->registerQuery(checkQuery);
//            this->checkQueries.emplace(horizon,checkQuery);
//        }



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
        std::cout << "call getSolution" << std::endl;
        sat = this->solver->getSolution(vars, terms, results);
                std::cout << "PROBLEM IS " << (sat ? "SATISFIABLE" : "NOT SATISFIABLE") << ", " << std::endl; // inquiryTerm->getQueryRule() << std::endl;
        ++horizon;


    }

    reasoner::asp::IncrementalExtensionQuery::cleanUp();

    //new run should start without active step queries
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

//    std::lock_guard<std::mutex> lock(mtx);
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
    auto sat = this->solver->getSolution(vars, terms, results);
        std::cout << "PROBLEM IS " << (sat ? "SATISFIABLE" : "NOT SATISFIABLE") << ", " << std::endl; // inquiryTerm->getQueryRule() << std::endl;

    for (auto res : results) {
        for (auto& varQueryValue : res->variableQueryValues) {
            for (const auto& elem : varQueryValue) {
                ret.push_back(elem);

            }
        }
    }

    return ret;
}

} /* namespace aspkb */