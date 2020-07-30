#include "aspkb/PossibleNextFieldGenerationProblem.h"

#include <aspkb/Extractor.h>
#include <memory>
#include <reasoner/asp/Variable.h>
#include <utility>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/model/Field.h>

namespace aspkb
{

PossibleNextFieldGenerationProblem::PossibleNextFieldGenerationProblem(::reasoner::asp::Solver* solver,
        const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& externalPrefix, int maxHorizon)
        : IncrementalProblem(solver, {"possibleNextCandidate(wildcard,wildcard)"}, baseTermProgramSectionParameters, "KnowledgeManager",
                  "possibleNextIncrementalProblem", externalPrefix, 0, maxHorizon, false)
{
}

std::vector<std::string> PossibleNextFieldGenerationProblem::doIncrementalSolving()
{

    // might be able to re-use safePathExits- rules here but with different check term?
    // external: start(X,Y) for visited field
    // base:  path(X,Y,0) :- start(X,Y).
    // step:
    // timestep(0..t).
    // path(X,Y,t) :- fieldAdjacent(X,Y,A,B), explored(X,Y), path(A,B,t-1), not path(X,Y,T), timestep(T). //duplicates should be avoided, right?
    // check:
    /*
        lastField(X,Y) :- path(X,Y,t-1).
        pathEndpoint(X,Y) :- lastField(X,Y), not explored(A,B) : fieldAdjacent(X,Y,A,B).
        alternative:
        pathEndpoint(X,Y) :- lastField(X,Y), not path(A,B,t), fieldAdjacent(X,Y,A,B).
        newFieldsCount :- N = #count{1,X,Y : path(X,Y,t)}
        :- newFieldsCount(0) // no more paths can be expanded, so stop

        TBD: how to collect results?
        example: last fields (1,1), (3,0), timestep 3
        // (3,0) is the end of a path, so lastField(3,0) and not fieldAdjacent(3,0,A,B), not explored(X,Y).
        then all unexplored fields adjacent to (3,0) are possibleNext
     */

    //

    std::vector<std::string> ret;
    auto vars = std::vector<::reasoner::asp::Variable*>();
    auto tmpVar = std::make_shared<::reasoner::asp::Variable>();
    vars.push_back(tmpVar.get());
    auto terms = std::vector<::reasoner::asp::Term*>();
    std::vector<::reasoner::asp::AnnotatedValVec*> results;
    bool sat = false;
    alica::AlicaTime timeElapsed = alica::AlicaTime::zero();
    int horizon = this->startHorizon;

    if (!this->keepBase) {
        this->activateBase();
    }
    //    std::cout << "activated base, sat is " << sat << " and maxhorizon " << this->maxHorizon << std::endl;
    while (!sat && horizon <= this->maxHorizon) {
        std::lock_guard<std::mutex> lock(TermManager::queryMtx);
        auto registeredQueries = this->solver->getRegisteredQueries(); // there are new queries added in each iteration
        terms.clear();

        if (horizon > 0) {
            this->deactivateCheck(horizon - 1);
            //            std::cout << "activate step for horizon" << horizon << std::endl;
            //        std::cout << "SOLVING BEFORE STEP TERM ACTIVATION" << std::endl;
            //        bool test = this->solver->getSolution(vars, terms, results);
            //        std::cout << "SOLVING BEFORE STEP TERM ACTIVATION RETURNED " << test << std::endl;
            this->activateStep(horizon);
            //        if (horizon > 1) {
            //            this->deactivateStep(horizon - 1);
            //        }
            //            std::cout << "activated step for horizon" << horizon << std::endl;
            //                    std::cout << "SOLVING BEFORE CHECK TERM ACTIVATION" << std::endl;
            //                    bool test2 = this->solver->getSolution(vars, terms, results);
            //                    std::cout << "SOLVING BEFORE CHECK TERM ACTIVATION RETURNED " << test2 << std::endl;

            this->activateCheckTerm(horizon);
            activateInquiries(terms, horizon);

            // add terms to terms passed in getSolution
            auto timeBefore = wumpus::WumpusWorldModel::getInstance()->getEngine()->getAlicaClock()->now();
            sat = this->solver->getSolution(vars, terms, results);
            auto solveTime = wumpus::WumpusWorldModel::getInstance()->getEngine()->getAlicaClock()->now() - timeBefore;
            timeElapsed = timeElapsed + solveTime;

            std::cout << "PROBLEM IS " << (sat ? "SATISFIABLE" : "NOT SATISFIABLE") << std::endl; // inquiryTerm->getQueryRule() << std::endl;
            ++horizon;
        } else { // horizon == 0, immediately check if target is met already without generating a step first
            this->activateCheckTerm(horizon);
            activateInquiries(terms, horizon);
            this->solver->getSolution(vars, terms, results);
            ++horizon;
        }
    }
    //    this->writeGetSolutionStatsIncremental(horizon, timeElapsed);

    // FIXME loop in reverse
    std::lock_guard<std::mutex> lock1(TermManager::queryMtx);

    this->falsifyAllStepTerms();
    this->deactivateAllChecks();

    if (!this->keepBase) {
        this->deactivateBase();
    }

    // TODO something is buggy here (way too many result entries)
    for (auto res : results) {
        for (auto& factQueryValue : res->factQueryValues) {
            for (auto elem : factQueryValue) {
                if (std::find(ret.begin(), ret.end(), elem) == ret.end()) {
                    //                    std::cout << "Result return: adding " << elem << std::endl;
                    ret.push_back(elem);
                } else {
                }
            }
        }
        delete res;
    }
    return ret;
}

void PossibleNextFieldGenerationProblem::activateInquiries(std::vector<::reasoner::asp::Term*>& terms, int horizon)
{ // Create Term belonging to a FilterQuery to be part of the terms in the getSolution call
    for (auto inquiry : inquiryPredicates) {
        for (int j = 0; j <= horizon; ++j) {
            if (j == 0) {
                bool found = false;
                for (const auto& i : inquiryPredicates) {
                    if (i.find("Complete") != std::string::npos) { //|| i.find("Candidate") != std::string::npos) {
                                                                   //                        std::cout << "found 'complete' in " << i << std::endl;
                        found = true;
                        break;
                    }
                }
                if (found) {
                    continue;
                }
            } // FIXME hack - review
              //            std::cout << "activate inquiry term " << inquiry << "," << j << std::endl;

            auto term = activateInquiryTerm(inquiry, j);
            // this should collect results - validate!
            terms.push_back(term);
        }
    }
}

void PossibleNextFieldGenerationProblem::deactivateStep(int i)
{
    this->incWrapper->deactivate(i);
}
}