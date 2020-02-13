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
        : isIntegrating(false)
{
    std::cout << "Creating Integrator" << std::endl;
    this->solver = TermManager::getInstance().getSolver();
    std::cout << "Created Integrator" << std::endl;
}
// TODO rework strategies
bool Integrator::integrateInformationAsExternal(std::string value, const std::string& identifier, bool truthValue, Strategy strategy = Strategy::INSERT_TRUE)
{
    this->setIsIntegrating(true);
    bool changed = false;
    ::reasoner::asp::Term* term;

    std::lock_guard<std::mutex> lock(this->mtx);
    if (this->identifierQueryMap.find(identifier) != this->identifierQueryMap.end()) {
        term = this->identifierQueryMap.at(identifier)->getTerm();
        auto ext = term->getExternals();
        bool found = false;
        for (auto& e : *ext) {
            // std::cout << "EXTERNAL: " << e.first << std::endl;
            if (strategy == Strategy::FALSIFY_OLD_VALUES && e.second) {
                e.second = false;
            }
            if (e.first == value) {
                found = true;
                if (e.second != truthValue) {
                    //                    std::cout << "changed " << value << " s truth value" << std::endl;
                    e.second = truthValue;
                    changed = true;
                }
            }
        }
        if (!found && !value.empty()) {
            ext->emplace(value, truthValue);
            //            std::cout << "couldn't find " << value << "so it was added" << std::endl;
            changed = true;
        }
    } else {
        term = TermManager::getInstance().requestTerm();
        term->setType(::reasoner::asp::QueryType::Extension);
        if (!value.empty()) {
            auto externals = term->getExternals();
            //            std::cout << "registered new query with identifier " << identifier << std::endl;
            changed = true;
            externals->emplace(value, true);
        }
        //        std::cout << "integrator: making extension query!" << std::endl;
        auto query = std::make_shared<::reasoner::asp::ExtensionQuery>(solver, term);
        this->solver->registerQuery(query);
        this->identifierQueryMap.emplace(identifier, query);
    }
    this->setIsIntegrating(false);
    return changed;
}

// TODO rename to addBackgroudKnowledge
void Integrator::integrateAsTermWithProgramSection(
        const std::string& programSection, const std::pair<std::vector<std::string>, std::vector<std::string>>& programSectionParameters)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    this->setIsIntegrating(true);
    ::reasoner::asp::Term* term = TermManager::getInstance().requestTerm();
    term->setBackgroundKnowledgeFilename(programSection);
    term->setProgramSection(programSection);
    for (auto representation : programSectionParameters.first) {
        for (auto param : programSectionParameters.second)
            term->addProgramSectionParameter(representation, param);
    }
    auto query = std::make_shared<::reasoner::asp::ExtensionQuery>(solver, term);
    this->solver->registerQuery(query);
    this->setIsIntegrating(false);
}

bool Integrator::getIsIntegrating()
{
    return this->isIntegrating;
}

void Integrator::setIsIntegrating(bool integrating)
{
    std::lock_guard<std::mutex> lock(this->integratingMtx);
    this->isIntegrating = integrating;
}

} /* namespace aspkb */
