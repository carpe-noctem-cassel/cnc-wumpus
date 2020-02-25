#include "aspkb/Integrator.h"
#include "aspkb/TermManager.h"
#include "wumpus/WumpusWorldModel.h"
#include <reasoner/asp/AnnotatedValVec.h>
#include <reasoner/asp/ExtensionQuery.h>
#include <reasoner/asp/Term.h>
#include <reasoner/asp/Variable.h>

namespace aspkb
{

std::mutex Integrator::mtx;
std::mutex Integrator::integratingMtx;

Integrator::Integrator()
{
    this->solver = TermManager::getInstance().getSolver();
    this->changedExternals = std::make_shared<std::map<std::string, bool>>();
}
// TODO rework strategies
void Integrator::integrateInformationAsExternal(
        const std::string& value, const std::string& identifier, bool truthValue, Strategy strategy = Strategy::INSERT_TRUE)
{
    //    this->setIsIntegrating(true);

    std::lock_guard<std::mutex> lock(aspkb::Integrator::mtx);
    // FIXME use truthvalue and strategy toggle instead of using empty values!
    if (!value.empty()) {
        this->changedExternals->emplace(value, truthValue);
    }

    if (strategy == Strategy::FALSIFY_OLD_VALUES) {
        if (this->identifierOldExternalsMap.find(identifier) != this->identifierOldExternalsMap.end()) {
            this->changedExternals->emplace(this->identifierOldExternalsMap.at(identifier), false);
            if (!value.empty()) {
                this->identifierOldExternalsMap.at(identifier) = value;
            }
        } else {
            if (!value.empty()) {
                this->identifierOldExternalsMap.emplace(identifier, value);
            }
        }
    }
}

void Integrator::applyChanges()
{
    std::lock_guard<std::mutex> lock(aspkb::Integrator::mtx);
    this->solver->handleExternals(this->changedExternals);
    this->changedExternals->clear();
}

// TODO rename to addBackgroudKnowledge
void Integrator::integrateAsTermWithProgramSection(
        const std::string& programSection, const std::pair<std::vector<std::string>, std::vector<std::string>>& programSectionParameters)
{
    ::reasoner::asp::Term* term = TermManager::getInstance().requestTerm();
    term->setBackgroundKnowledgeFilename(programSection);
    term->setProgramSection(programSection);
    for (const auto& representation : programSectionParameters.first) {
        for (const auto& param : programSectionParameters.second)
            term->addProgramSectionParameter(representation, param);
    }
    {
        std::lock_guard<std::mutex> queryLock(TermManager::queryMtx);
        auto query = std::make_shared<::reasoner::asp::ExtensionQuery>(solver, term);
        this->solver->registerQuery(query);
    }
}

} /* namespace aspkb */
