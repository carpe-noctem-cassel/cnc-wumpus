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

    bool integrateInformationAsExternal(std::string value, const std::string& identifier, bool truthValue, aspkb::Strategy strategy);

    void integrateAsTermWithProgramSection(
            const std::string& programSection, const std::pair<std::vector<std::string>, std::vector<std::string>>& programSectionParameters);

    bool getIsIntegrating();
    void setIsIntegrating(bool integrating);

private:
    ::reasoner::asp::Solver* solver;

    std::map<std::string, std::shared_ptr<::reasoner::asp::ExtensionQuery>> identifierQueryMap;

    bool isIntegrating;

    static std::mutex mtx;
    static std::mutex integratingMtx;
};

} /* namespace aspkb */
