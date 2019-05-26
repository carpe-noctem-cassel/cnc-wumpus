#pragma once
#include "Strategy.h"
#include "TermManager.h"
#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <engine/AlicaEngine.h>
#include <mutex>
#include <reasoner/asp/ExtensionQuery.h>
#include <reasoner/asp/Solver.h>
#include <vector>

namespace aspkb
{
class Integrator
{
public:
    Integrator();

    virtual ~Integrator() = default;

    bool integrateInformationAsExternal(const std::string& value, const std::string& identifier, aspkb::Strategy strategy);

    void integrateAsTermWithProgramSection(
            const std::string& programSection, const std::pair<std::vector<std::string>, std::vector<std::string>>& programSectionParameters);

private:
    ::reasoner::asp::Solver* solver;

    std::map<std::string, ::reasoner::asp::ExtensionQuery*> identifierQueryMap;

    std::mutex mtx;
};

} /* namespace aspkb */
