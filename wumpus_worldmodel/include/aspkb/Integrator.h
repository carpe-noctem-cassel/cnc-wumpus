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

    void integrateInformationAsExternal(const std::string& value, const std::string& identifier, bool truthValue, aspkb::Strategy strategy);

    void integrateAsTermWithProgramSection(
            const std::string& programSection, const std::pair<std::vector<std::string>, std::vector<std::string>>& programSectionParameters);

    void applyChanges();

    static std::mutex integratingMtx;

private:
    ::reasoner::asp::Solver* solver;

    /**
     * Maps predicate names (e.g. "on") to the last occurred value (e.g. on(1,0)) if the integration strategy has to falsify old values
     * TODO: can situations happen where multiple old values need to be falsified at once?
     */
    std::map<std::string, std::string> identifierOldExternalsMap;
    
    /**
     * Changes to apply to externals in next planning iteration
     */
    std::shared_ptr<std::map<std::string, bool>> changedExternals;

    static std::mutex mtx;
};

} /* namespace aspkb */
