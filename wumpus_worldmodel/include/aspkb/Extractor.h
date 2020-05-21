#pragma once
#include "Strategy.h"
#include "TermManager.h"
#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <mutex>
#include <reasoner/asp/ExtensionQuery.h>
#include <reasoner/asp/Solver.h>
#include <set>
#include <vector>

namespace wumpus
{
class WumpusWorldModel;
}
namespace aspkb
{
class IncrementalProblem;
class Extractor
{
public:
    Extractor();

    ~Extractor();

    std::vector<std::string> extractReusableTemporaryQueryResult(
            const std::vector<std::string>& inquiryPredicates, const std::string& queryIdentifier, const std::vector<std::string>& additionalRules);

    std::vector<std::string> solveWithIncrementalExtensionQuery(const std::shared_ptr<aspkb::IncrementalProblem>& inc);

private:
    ::reasoner::asp::Solver* solver;

    std::map<int, std::shared_ptr<::reasoner::asp::ReusableExtensionQuery>> checkQueries; // TODO make list and imply ordering by horizon?
    std::map<int, ::reasoner::asp::Term*> checkTerms;

    void writeGetSolutionStatsIncremental(int horizon, alica::AlicaTime timeElapsed);
    void writeGetSolutionStatsReusable(const std::string& queryIdentifier, alica::AlicaTime timeElapsed);
    inline void writeHeader(bool incremental);

    static std::mutex mtx;

    static std::mutex incrementalMtx;
    static std::mutex reusableMtx;

    std::string resultsDirectory;
    std::string filenameIncremental;
    std::string filenameReusable;

    static bool wroteHeaderIncremental;

    static bool wroteHeaderReusable;
};

} /* namespace wm */
