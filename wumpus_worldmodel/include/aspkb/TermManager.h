#pragma once
#include <reasoner/asp/ReusableExtensionQuery.h>
#include <reasoner/asp/Solver.h>
#include <mutex>

namespace reasoner
{
namespace asp
{
class  ReusableExtensionQuery;
}
}
namespace aspkb
{
class TermManager
{

public:
    static TermManager& getInstance();

    static std::mutex queryMtx;

    ::reasoner::asp::Term* requestTerm();

    ::reasoner::asp::Term* requestCheckTerm(int horizon);

    ::reasoner::asp::Term* requestPathCheckTerm(int horizon, std::string coordinateEncoding);

    int activateReusableExtensionQuery(std::string identifier, const std::vector<std::string>& rules);
    void deactivateReusableExtensionQuery(std::string identifier);

    void initializeSolver(::reasoner::asp::Solver* solver);

    /**
     * Provides solver for Integrator & Extractor
     * @return
     */
    ::reasoner::asp::Solver* getSolver() const;

    TermManager(TermManager const&) = delete;
    void operator=(TermManager const&) = delete;

    //TODO remove, only to make evaluation easier
    void clear();

private:
    TermManager() {}
    ~TermManager();

    std::vector<::reasoner::asp::Term*> managedTerms;
    std::map<std::string, std::shared_ptr<::reasoner::asp::ReusableExtensionQuery>> reusableQueries;
    ::reasoner::asp::Solver* solver;
    static std::mutex mtx;
};
}
