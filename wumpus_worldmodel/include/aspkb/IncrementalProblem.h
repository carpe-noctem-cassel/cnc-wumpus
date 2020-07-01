#pragma once

#include <reasoner/asp/IncrementalQueryWrapper.h>

#include <map>
#include <string>
#include <vector>

namespace reasoner
{
namespace asp
{
class Term;
}
}
namespace essentials
{
class SystemConfig;
}
namespace aspkb
{
class IncrementalProblem
{
    const char* const BASE_RULES_CONFIG = "baseRulesFilePath";
    const char* const STEP_RULES_CONFIG = "stepRulesFilePath";
    const char* const CHECK_RULES_CONFIG = "checkRulesFilePath";

public:
    IncrementalProblem(::reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
            const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename,
            const std::string& configurationSection, const std::string& externalPrefix, int startHorizon, int maxHorizon, bool keepBase);

    const std::vector<std::string> inquiryPredicates;

    ::reasoner::asp::Term* baseTerm;
    std::map<int, ::reasoner::asp::Term*> stepTermsByHorizon;
    std::map<int, ::reasoner::asp::Term*> checkTermsByHorizon;
    std::map<std::string, std::map<int, ::reasoner::asp::Term*>> inquiryTermsByHorizon;

    const int maxHorizon;
    const int startHorizon;
    const bool keepBase;

    std::vector<std::string> stepRules;

    virtual void activateStep(int horizon);
    void activateCheckTerm(int horizon);
    void deactivateCheck(int horizon);
    void deactivateAllChecks();
    void addStep(int horizon);
    void addCheck(int horizon);
    void falsifyAllStepTerms();
    void activateBase();
    void deactivateBase();


   ::reasoner::asp::Term* activateInquiryTerm(const std::string& inquiryString, int horizon);

protected:
    IncrementalProblem(::reasoner::asp::Solver* solver, std::vector<std::string> inquiryPredicates,
            const std::map<std::string, std::string>& baseTermProgramSectionParameters, const std::string& configFilename,
            const std::string& configurationSection, int startHorizon, int maxHorizon, bool keepBase,
            std::shared_ptr<::reasoner::asp::IncrementalQueryWrapper> wrapper);

   ::reasoner::asp::Solver* solver;
    std::shared_ptr<::reasoner::asp::IncrementalQueryWrapper> incWrapper;
private:
    std::vector<std::string> baseRules;
    std::vector<std::string> checkRules;

    void loadAdditionalRules(const std::string& filePath, std::vector<std::string>& ruleContainer) const;

    ::reasoner::asp::Term* createNewInquiryTerm(const std::string& inquiryString, int horizon) const;

    std::string wrapWithPrefixForHorizon(const std::string& predicate, int horizon);
};
}