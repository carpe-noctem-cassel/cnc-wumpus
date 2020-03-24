#pragma once

#include <WumpusEnums.h>
#include <map>
#include <memory>
#include <reasoner/asp/PredicateContainer.h>
#include <reasoner/asp/SyntaxUtils.h>
#include <string>
#include <utility> //pair
#include <vector>
#include <wumpus/model/Objective.h>
#include <mutex>
#include <set>

namespace aspkb
{
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

public:
    PlanningModule(wumpus::WumpusWorldModel* wm);

    ~PlanningModule();

    std::pair<int, std::vector<WumpusEnums::actions>> processNextActionRequest(std::shared_ptr<wumpus::model::Agent> agent);

    std::string determineGoal();

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> determinePathAndActions(
            const std::vector<std::string>& generationRules, int horizon);

    std::vector<std::string> determineActionsNoMovement();

    bool determineWumpusBlocksSafeMoves();

    bool determineAllAgentsExhausted();

    bool determinePosToShootFrom();

    wumpus::model::Objective determineObjective();

    std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> shootingTargets; //FIXME


    bool getIsPlanning();
    void setIsPlanning(bool planning);

    void addShootingTarget(const int id, const std::pair<std::basic_string<char>, std::basic_string<char>>& shotAt);
    std::shared_ptr<std::map<int, std::set<std::pair<std::string, std::string>>>> getShootingTargets();
    void addBlockingWumpusOfOther(const std::pair<std::basic_string<char>, std::basic_string<char>>& wumpus);
    void addBlockingWumpusOfSelf(const std::pair<std::basic_string<char>, std::basic_string<char>>& wumpus);

private:
    essentials::SystemConfig* sc;
    wumpus::WumpusWorldModel* wm;
    aspkb::Extractor* extractor;

    void loadAdditionalRules(const std::string& filePath, std::vector<std::string>& ruleContainer);

    std::string lastGoal;
    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> lastPathAndActions;

    std::pair<std::string, std::string> extractBinaryPredicateParameters(const std::string& str, const std::string& predName);
    std::string extractUnaryPredicateParameter(const std::string& str);



    double maxHorizonFactor;

    std::vector<std::string> goalGenerationRules;
    std::vector<std::string> pathValidationRules;
    std::vector<std::string> actionsGenerationRules;
    std::vector<std::string> actionsGenWumpusRules;
    std::vector<std::string> objectiveRules;
    std::vector<std::string> shootFromRules;
    std::vector<std::string> allAgentsExhaustedRules;

    // experimental
    std::vector<std::string> baseRules;
    std::vector<std::string> stepRules;
    std::vector<std::string> checkRules;

    std::vector<std::pair<std::string,std::string>> blockingWumpiOfOthers;
    std::vector<std::pair<std::string,std::string>> blockingWumpiOfSelf;

    std::mutex shotAtMtx;
    std::mutex wumpusMtxOther;
    std::mutex wumpusMtxSelf;
    std::mutex planningMtx;

    bool isPlanning;


    bool agentObjectiveRequiresMovement(const std::shared_ptr<wumpus::model::Agent> &agent) const;

    bool agentObjectiveRequiresGoal(const std::shared_ptr<wumpus::model::Agent> &agent) const;

    bool objectiveImpliesSimpleAction(const std::shared_ptr<model::Agent> &agent) const;

    std::pair<std::vector<std::pair<std::string, std::string>>, std::vector<std::string>> tryGetSafeActions();

};
} /*namespace wm */
} /*namespace wumpus*/
