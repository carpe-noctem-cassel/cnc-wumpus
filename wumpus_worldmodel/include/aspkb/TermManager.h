#pragma once
#include <reasoner/asp/ReusableExtensionQuery.h>
#include <reasoner/asp/Solver.h>

namespace reasoner
{
namespace asp
{
class ReusableExtensionQuery;
}
}
namespace aspkb
{
class TermManager
{

public:
    static TermManager& getInstance();

    ::reasoner::asp::Term* requestTerm();

    ::reasoner::asp::Term* requestCheckTerm(int horizon);

    int activateReusableExtensionQuery(std::string identifier, const std::vector<std::string>& rules);

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
    std::map<std::string, ::reasoner::asp::ReusableExtensionQuery*> reusableQueries;
    ::reasoner::asp::Solver* solver;
};
}
