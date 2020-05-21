#pragma once

#include <WumpusEnums.h>
#include <map>
#include <memory>
#include <mutex>
#include <reasoner/asp/PredicateContainer.h>
#include <reasoner/asp/SyntaxUtils.h>
#include <set>
#include <string>
#include <utility> //pair
#include <vector>
#include <wumpus/model/Objective.h>

namespace aspkb
{
class IncrementalProblem;
class Extractor;
}
namespace essentials
{
class SystemConfig;
}
namespace wumpus
{
class WumpusWorldModel;
namespace model
{
class Agent;
class Field;
}
namespace wm
{
class PlanningModule
{
    const char* const KB_CONFIG_NAME = "KnowledgeManager";

public:
    PlanningModule(wumpus::WumpusWorldModel* wm);

    ~PlanningModule();

    std::pair<int, std::vector<WumpusEnums::actions>> processNextActionRequest(std::shared_ptr<wumpus::model::Agent> agent);

    std::string determineGoal();

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> determinePathAndActions();

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> determineActionsNoMovement();

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> tryDetermineActionsByObjective(
            const std::shared_ptr<wumpus::model::Agent>& agent);

    std::shared_ptr<aspkb::IncrementalProblem> getSafePathProblem();

    bool determineWumpusBlocksSafeMoves();

    bool determineAllAgentsExhausted();

    void checkSafePathsExistsForOtherAgents(std::pair<int, int> from, const std::map<int, std::pair<int, int>>& tos);

    std::pair<std::pair<std::string,std::string>,std::string> determinePosToShootFrom();

    wumpus::model::Objective determineObjective();

    std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> shootingTargets; // FIXME

    bool getIsPlanning();
    void setIsPlanning(bool planning);

    void addShootingTarget(int id, const std::pair<std::basic_string<char>, std::basic_string<char>>& shotAt);
    std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> getShootingTargets();
    void addBlockingWumpusOfOther(const std::pair<std::basic_string<char>, std::basic_string<char>>& wumpus);
    void addBlockingWumpusOfSelf(const std::pair<std::basic_string<char>, std::basic_string<char>>& wumpus);

private:
    essentials::SystemConfig* sc;
    wumpus::WumpusWorldModel* wm;
    aspkb::Extractor* extractor;

    static void loadAdditionalRules(const std::string& filePath, std::vector<std::string>& ruleContainer);

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> lastPathAndActions;

    static std::pair<std::string, std::string> extractBinaryPredicateParameters(const std::string& str, const std::string& predName);
    static std::string extractUnaryPredicateParameter(const std::string& str);

    double maxHorizonFactor;

    std::vector<std::string> goalGenerationRules;
    std::vector<std::string> pathValidationRules;
    std::vector<std::string> actionsGenerationRules;
    std::vector<std::string> actionsGenWumpusRules;
    std::vector<std::string> objectiveRules;
    std::vector<std::string> shootFromRules;
    std::vector<std::string> allAgentsExhaustedRules;

    // experimental
    std::shared_ptr<aspkb::IncrementalProblem> pathAndActionsProblem;
    std::shared_ptr<aspkb::IncrementalProblem> safePathExistsForOtherAgentProblem;
    std::map<std::pair<int, int>, std::map<std::pair<int, int>, std::shared_ptr<aspkb::IncrementalProblem>>> safePathExistsForOtherAgentProblems;

    std::vector<std::pair<std::string, std::string>> blockingWumpiOfOthers;
    std::vector<std::pair<std::string, std::string>> blockingWumpiOfSelf;

    std::mutex shotAtMtx;
    std::mutex wumpusMtxOther;
    std::mutex wumpusMtxSelf;
    std::mutex planningMtx;

    bool isPlanning;

    bool agentObjectiveRequiresMovement(const std::shared_ptr<wumpus::model::Agent>& agent) const;

    bool agentObjectiveRequiresGoal(const std::shared_ptr<wumpus::model::Agent>& agent) const;

    bool objectiveImpliesSimpleAction(const std::shared_ptr<model::Agent>& agent) const;

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> tryGetSafeActions(const std::shared_ptr<wumpus::model::Agent>& agent);
};
} /*namespace wm */
} /*namespace wumpus*/
