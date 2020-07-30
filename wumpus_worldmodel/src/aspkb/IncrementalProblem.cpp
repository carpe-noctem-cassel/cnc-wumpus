#include "aspkb/IncrementalProblem.h"
#include <SystemConfig.h>

#include <aspkb/TermManager.h>
#include <utility>

namespace aspkb
{
std::mutex IncrementalProblem::incProblemMtx;
IncrementalProblem::IncrementalProblem(::reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
        const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename, const std::string& configSection,
        int startHorizon, int maxHorizon, bool keepBase, std::shared_ptr<::reasoner::asp::IncrementalQueryWrapper> wrapper)
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

    if (incWrapper->getQueryExternalPrefix().find("possibleNext") != std::string::npos) {
        //        throw std::exception();
    }

    // register base term
    this->baseTerm = TermManager::getInstance().requestTerm();
    baseTerm->setLifeTime(-1);
    baseTerm->setType(::reasoner::asp::QueryType::IncrementalExtension);
    for (const auto& param : baseTermProgramSectionParameters) {
        baseTerm->addProgramSectionParameter(param.first, param.second);
    }
    for (const auto& str : baseRules) {
        baseTerm->addRule(str);
    }
    std::cout << "incproblem: base term has id " << baseTerm->getQueryId()  << " for inqueries " << std::endl;
    for(auto in : this->inquiryPredicates) {
        std::cout << in << std::endl;
        std::cout << this->incWrapper->getQueryExternalPrefix() << std::endl;
    }

    std::lock_guard<std::mutex> lock(incProblemMtx);
    this->incWrapper->addQueryForHorizon(0, baseTerm);
    //    auto query = std::make_shared<::reasoner::asp::IncrementalExtensionQuery>(this->solver, baseTerm, this->incWrapper->getQueryExternalPrefix(), 0);
    //    this->solver->registerQuery(query);
    //    this->incWrapper->addQueryForHorizon(0, baseTerm);
}

IncrementalProblem::IncrementalProblem(::reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
        const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename, const std::string& configSection,
        const std::string& externalPrefix, int startHorizon, int maxHorizon, bool keepBase)
        : IncrementalProblem(solver, std::move(inquiryPredicates), baseTermProgramSectionParameters, configFilename, configSection, startHorizon, maxHorizon,
                  keepBase, std::make_shared<::reasoner::asp::IncrementalQueryWrapper>(solver, externalPrefix))
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
    stepTerm->setType(::reasoner::asp::QueryType::IncrementalExtension);
    stepTerm->addProgramSectionParameter("t", std::to_string(horizon));
    for (const auto& str : this->stepRules) {
        stepTerm->addRule(str);
    }
    // manually added rules are not expanded properly :( FIXME
    if (this->incWrapper->getQueryExternalPrefix() == "pathActions") {
        stepTerm->addRule("{occurs(A,t-1) : moveAction(A)} = 1.");
    } else if (this->incWrapper->getQueryExternalPrefix().find("possibleNext") != std::string::npos) {
        //        auto rule = "lastField(X,Y) :- " + wrapWithPrefixForHorizon("genPath(X,Y,t-1)", horizon - 1) + ".";
        //        stepTerm->addRule(rule);
        auto rule =
                "{genPath(X, Y, t) : fieldAdjacent(X, Y, A, B), explored(X, Y)," + wrapWithPrefixForHorizon("genPath(A, B, t - 1)", horizon - 1) + "} <= 1. ";
        // FIXME >= and weak constraint????
        stepTerm->addRule(rule);
        //        rule = "genPathEndpoint(X,Y) :- lastField(X,Y), not explored(A,B) : fieldAdjacent(X,Y,A,B), lastField(X,Y).";
        //        stepTerm->addRule(rule);

        for (int i = 0; i < horizon; ++i) { // FIXME was < before
            rule = ":- genPath(X, Y, t), " + wrapWithPrefixForHorizon("genPath(X, Y, T)", i) + ", timestep(T), T < t.";
            stepTerm->addRule(rule);
            auto predicate = "genPath(X,Y," + std::to_string(i) + ")";
            rule = "genPathEndpoint(X,Y) :- " + this->incWrapper->getQueryExternalPrefix() + "incquery" + std::to_string(i) + "(genPath(X,Y," +
                   std::to_string(i) + ")) , fieldAdjacent(X,Y,A,B), not explored(A,B)";
            stepTerm->addRule(rule);
        }

        rule = ":~ not " + wrapWithPrefixForHorizon("genPath(X,Y,t)", horizon) + ", field(X,Y).[1@1]"; // TODO :~ not genPath(X,Y,t). wrapped
        stepTerm->addRule(rule);
        //        rule = "possibleNextCandidate(X,Y) :- " + wrapWithPrefixForHorizon("genPathEndPoint(A,B)",horizon) + ", fieldAdjacent(X,Y,A,B), not
        //        explored(X,Y).";
        //        stepTerm->addRule(rule);
        //        rule = "genPathEndpoint(X,Y) :- " + this->incWrapper->getQueryExternalPrefix() + "incquery" + std::to_string(horizon) + "(lastField(X,Y))" +
        //        ", not " +
        //               wrapWithPrefixForHorizon("genPath(X,Y," + std::to_string(horizon) + ")", horizon) + " : fieldAdjacent(X,Y,A,B), lastField(X,Y).";
        //        stepTerm->addRule(rule);
    }
    //    for (auto r : stepTerm->getRules()) {
    //        std::cout << "stepTerm rule: " << r << std::endl;
    //    }
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
    std::lock_guard<std::mutex> lock(incProblemMtx);

    if (this->checkTermsByHorizon.find(horizon) != this->checkTermsByHorizon.end()) {
        auto registeredQueries = this->solver->getRegisteredQueries();
        for (const auto& query : registeredQueries) {
            if (query->getTerm()->getId() == this->checkTermsByHorizon.at(horizon)->getId()) {
                std::dynamic_pointer_cast<::reasoner::asp::ReusableExtensionQuery>(query)->reactivate();
                break;
            }
        }

    } else {
        this->addCheck(horizon);
    }
}

void IncrementalProblem::deactivateCheck(int horizon)
{
    std::lock_guard<std::mutex> lock(incProblemMtx);

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
    ::reasoner::asp::Term* checkTerm;
    if (this->incWrapper->getQueryExternalPrefix() == "pathActions") {
        checkTerm = aspkb::TermManager::getInstance().requestCheckTerm(horizon);
    } else if (this->incWrapper->getQueryExternalPrefix().find("safePath") != std::string::npos) {
        checkTerm = aspkb::TermManager::getInstance().requestPathCheckTerm(
                horizon, this->incWrapper->getQueryExternalPrefix().substr(this->incWrapper->getQueryExternalPrefix().find("safePath")));
    } else if (this->incWrapper->getQueryExternalPrefix().find("possibleNext") != std::string::npos) {
        checkTerm = aspkb::TermManager::getInstance().requestPossibleNextCheckTerm(this->incWrapper->getQueryExternalPrefix(), horizon);
    } else {
        std::cerr << "IncrementalProblem: No suitable check term to create!" << std::endl;
        throw std::exception();
    }
    std::cout << "adding check term for horizon " << std::to_string(horizon) << "!" << std::endl;
    this->checkTermsByHorizon.emplace(horizon, checkTerm);
    auto query = std::make_shared<::reasoner::asp::ReusableExtensionQuery>(this->solver, checkTerm);
    std::cout << "made query" << std::endl;
    solver->registerQuery(query);
}

::reasoner::asp::Term* IncrementalProblem::activateInquiryTerm(const std::string& inquiryString, int horizon)
{
    if (this->inquiryTermsByHorizon.find(inquiryString) != this->inquiryTermsByHorizon.end()) {
        std::cout << "inquiryTermsByHorizon at " << inquiryString << std::endl;
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

::reasoner::asp::Term* IncrementalProblem::createNewInquiryTerm(const std::string& inquiryString, int horizon) const
{
    auto inquiryTerm = TermManager::getInstance().requestTerm();

    inquiryTerm->setType(::reasoner::asp::Filter);
    //                // wrap query rule to match extension query
    // FIXME save query external prefix locally?
    // FIXME parameter whether query rule needs to be wrapped
    if (this->incWrapper->getQueryExternalPrefix().find("safePath") == std::string::npos &&
            this->incWrapper->getQueryExternalPrefix().find("possibleNext") == std::string::npos) {
        auto wrappedQueryRule = this->incWrapper->getQueryExternalPrefix() + "incquery" + std::to_string(horizon) + "(" + inquiryString + ")";
        inquiryTerm->setQueryRule(wrappedQueryRule);
    } else {
        auto checkTermId = this->checkTermsByHorizon.at(horizon)->getId();
        auto wrappedQueryRule = "query" + std::to_string(checkTermId) + "(" + inquiryString + ")";
        std::cout << "Wrapped query rule is: " << wrappedQueryRule << std::endl;
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
    std::lock_guard<std::mutex> lock(incProblemMtx);
    std::cout << "activate base " << this->baseTerm->getQueryId() << ", " << this->incWrapper->getQueryExternalPrefix() << std::endl;
    this->incWrapper->activate(0);
    //    this->incWrapper->activate(0);
    //    std::cout << "Activate base: get registered queries" << std::endl;
    //    auto registeredQueries = this->solver->getRegisteredQueries2();
    //    std::cout << "Activate base: queryid" << this->baseTerm->getQueryId() << std::endl;
    //    for (auto query : registeredQueries) {
    //        std::cout << "Activate base: queryId in query: " << query->getTerm()->getQueryId() << std::endl;
    //        if (query->getTerm()->getQueryId() == this->baseTerm->getQueryId()) {
    //            this->solver->assignExternal(*(std::dynamic_pointer_cast<::reasoner::asp::IncrementalExtensionQuery>(query)->external),
    //            Clingo::TruthValue::True);
    //            break;
    //        }
    //    }
}

void IncrementalProblem::deactivateBase()
{
    this->incWrapper->deactivate(0);
    //    std::lock_guard<std::mutex> lock(incProblemMtx);
    //    auto registeredQueries = this->solver->getRegisteredQueries();
    //    for (const auto& query : registeredQueries) {
    //        if (query->getTerm()->getQueryId() == this->baseTerm->getQueryId()) {
    //            query->removeExternal();
    //            break;
    //        }
    //    }
}

std::string IncrementalProblem::wrapWithPrefixForHorizon(const std::string& predicate, int horizon)
{

    std::stringstream rule;
    rule << this->incWrapper->getQueryExternalPrefix() << "incquery" << horizon << "(" << predicate << ")";
    return rule.str();
}
}
