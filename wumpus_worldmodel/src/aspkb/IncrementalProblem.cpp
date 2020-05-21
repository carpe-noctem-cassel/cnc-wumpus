#include "aspkb/IncrementalProblem.h"
#include <SystemConfig.h>

#include <aspkb/TermManager.h>
#include <utility>

namespace aspkb
{
IncrementalProblem::IncrementalProblem(reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
        const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename, const std::string& configSection,
        int startHorizon, int maxHorizon, bool keepBase, std::shared_ptr<reasoner::asp::IncrementalQueryWrapper> wrapper)
        : startHorizon(startHorizon)
        , maxHorizon(maxHorizon)
        , keepBase(keepBase)
        , inquiryPredicates(std::move(inquiryPredicates))
        , solver(solver)
        , incWrapper(std::move(wrapper))
{

    auto sc = essentials::SystemConfig::getInstance();
    auto config = (*sc)[configFilename];

    auto filePath = config->get<std::string>((configSection + "." + STEP_RULES_CONFIG).c_str(), NULL);
    this->loadAdditionalRules(filePath, this->stepRules);
    filePath = config->get<std::string>((configSection + "." + BASE_RULES_CONFIG).c_str(), NULL);
    this->loadAdditionalRules(filePath, this->baseRules);
    filePath = config->get<std::string>((configSection + "." + CHECK_RULES_CONFIG).c_str(), NULL);
    this->loadAdditionalRules(filePath, this->checkRules);

    // register base term
    this->baseTerm = TermManager::getInstance().requestTerm();
    baseTerm->setLifeTime(-1);
    baseTerm->setType(reasoner::asp::QueryType::IncrementalExtension);
    for (const auto& param : baseTermProgramSectionParameters) {
        baseTerm->addProgramSectionParameter(param.first, param.second);
    }
    for (const auto& str : baseRules) {
        baseTerm->addRule(str);
    }
    auto query = std::make_shared<reasoner::asp::IncrementalExtensionQuery>(this->solver, baseTerm, this->incWrapper->getQueryExternalPrefix(), 0);
    this->solver->registerQuery(query);
    //    this->incWrapper->addQueryForHorizon(0, baseTerm);
}

IncrementalProblem::IncrementalProblem(reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
        const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename, const std::string& configSection,
        const std::string& externalPrefix, int startHorizon, int maxHorizon, bool keepBase)
        : IncrementalProblem(solver, std::move(inquiryPredicates), baseTermProgramSectionParameters, configFilename, configSection, startHorizon, maxHorizon,
                  keepBase, std::make_shared<reasoner::asp::IncrementalQueryWrapper>(solver, externalPrefix))
{
}

/**
FIXME resolve duplication
*/
void IncrementalProblem::loadAdditionalRules(const std::string& filePath, std::vector<std::string>& ruleContainer) const
{
    auto path2etc = std::getenv("DOMAIN_CONFIG_FOLDER");
    std::ifstream input(std::string(path2etc) + '/' + filePath);
    std::string line;
    while (std::getline(input, line)) {
        ruleContainer.emplace_back(line);
    }
}

void IncrementalProblem::addStep(int horizon)
{
    auto stepTerm = TermManager::getInstance().requestTerm();
    stepTerm->setLifeTime(-1);
    stepTerm->setType(reasoner::asp::QueryType::IncrementalExtension);
    stepTerm->addProgramSectionParameter("t", std::to_string(horizon));
    for (const auto& str : this->stepRules) {
        stepTerm->addRule(str);
    }
    // manually added rules are not expanded properly :( FIXME
    if (this->incWrapper->getQueryExternalPrefix() == "pathActions") {
        stepTerm->addRule("{occurs(A,t-1) : moveAction(A)} = 1.");
    }
    this->incWrapper->addQueryForHorizon(horizon, stepTerm);
}

void IncrementalProblem::activateStep(int horizon)
{
    if (this->incWrapper->isPresent(horizon)) {
        this->incWrapper->activate(horizon);
    } else {
        this->addStep(horizon);
    }
}

void IncrementalProblem::activateCheckTerm(int horizon)
{

    if (this->checkTermsByHorizon.find(horizon) != this->checkTermsByHorizon.end()) {
        auto registeredQueries = this->solver->getRegisteredQueries();
        for (const auto& query : registeredQueries) {
            if (query->getTerm()->getId() == this->checkTermsByHorizon.at(horizon)->getId()) {
                std::dynamic_pointer_cast<reasoner::asp::ReusableExtensionQuery>(query)->reactivate();
                break;
            }
        }

    } else {
        this->addCheck(horizon);
    }
}

void IncrementalProblem::deactivateCheck(int horizon)
{
    auto registeredQueries = this->solver->getRegisteredQueries();
    for (const auto& query : registeredQueries) {
        if (query->getTerm()->getQueryId() == this->checkTermsByHorizon.at(horizon)->getQueryId()) {
            query->removeExternal();
            break;
        }
    }
}

/**
 * FIXME make this work properly without checking prefix by loading checkRules
 * @param horizon
 */
void IncrementalProblem::addCheck(int horizon)
{
    reasoner::asp::Term* checkTerm;
    if (this->incWrapper->getQueryExternalPrefix() == "pathActions") {
        checkTerm = aspkb::TermManager::getInstance().requestCheckTerm(horizon);
    } else if (this->incWrapper->getQueryExternalPrefix().find("safePath") != std::string::npos) {
        checkTerm = aspkb::TermManager::getInstance().requestPathCheckTerm(
                horizon, this->incWrapper->getQueryExternalPrefix().substr(this->incWrapper->getQueryExternalPrefix().find("safePath")));
    } else {
        std::cerr << "IncrementalProblem: No suitable check term to create!" << std::endl;
        throw std::exception();
    }
    std::cout << "adding check term for horizon " << std::to_string(horizon) << "!" << std::endl;
    this->checkTermsByHorizon.emplace(horizon, checkTerm);
    auto query = std::make_shared<reasoner::asp::ReusableExtensionQuery>(this->solver, checkTerm);
    solver->registerQuery(query);
}

reasoner::asp::Term* IncrementalProblem::activateInquiryTerm(const std::string& inquiryString, int horizon)
{
    if (this->inquiryTermsByHorizon.find(inquiryString) != this->inquiryTermsByHorizon.end()) {
        auto terms = this->inquiryTermsByHorizon.at(inquiryString);
        if (terms.find(horizon) != terms.end()) {
            return terms.at(horizon);
        } else {
            return createNewInquiryTerm(inquiryString, horizon);
        }
    } else {
        return createNewInquiryTerm(inquiryString, horizon);
    }
}

reasoner::asp::Term* IncrementalProblem::createNewInquiryTerm(const std::string& inquiryString, int horizon) const
{
    auto inquiryTerm = TermManager::getInstance().requestTerm();

    inquiryTerm->setType(reasoner::asp::Filter);
    //                // wrap query rule to match extension query
    // FIXME save query external prefix locally?
    // FIXME parameter whether query rule needs to be wrapped
    if (this->incWrapper->getQueryExternalPrefix().find("safePath") == std::string::npos) {
        auto wrappedQueryRule = this->incWrapper->getQueryExternalPrefix() + "incquery" + std::to_string(horizon) + "(" + inquiryString + ")";
        inquiryTerm->setQueryRule(wrappedQueryRule);
    } else {
        for(const auto& entry : this->checkTermsByHorizon) {
        }
        auto checkTermId = this->checkTermsByHorizon.at(horizon)->getId();
        auto wrappedQueryRule = "query" + std::to_string(checkTermId) + "(" + inquiryString + ")";
        inquiryTerm->setQueryRule(wrappedQueryRule);
    }
    return inquiryTerm;
}

void IncrementalProblem::falsifyAllStepTerms()
{
    this->incWrapper->cleanUp();
}

void IncrementalProblem::deactivateAllChecks()
{
    for (const auto& element : this->checkTermsByHorizon) {
        this->deactivateCheck(element.first);
    }
}

void IncrementalProblem::activateBase()
{
    //    this->incWrapper->activate(0);
    auto registeredQueries = this->solver->getRegisteredQueries();
    for (const auto& query : registeredQueries) {
        if (query->getTerm()->getQueryId() == this->baseTerm->getQueryId()) {
            this->solver->assignExternal(*std::dynamic_pointer_cast<reasoner::asp::IncrementalExtensionQuery>(query)->external, Clingo::TruthValue::True);
            break;
        }
    }
}

void IncrementalProblem::deactivateBase()
{
    auto registeredQueries = this->solver->getRegisteredQueries();
    for (const auto& query : registeredQueries) {
        if (query->getTerm()->getQueryId() == this->baseTerm->getQueryId()) {
            query->removeExternal();
            break;
        }
    }
}
}
