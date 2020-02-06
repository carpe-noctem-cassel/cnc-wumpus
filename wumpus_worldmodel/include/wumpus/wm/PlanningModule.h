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

    void determinePosToShootFrom();

    wumpus::model::Objective determineObjective();

    std::vector<std::pair<std::string, std::string>> shootingTargets; //FIXME


    bool getIsPlanning();
    void setIsPlanning(bool planning);

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

    // experimental
    std::vector<std::string> baseRules;
    std::vector<std::string> stepRules;
    std::vector<std::string> checkRules;

    std::mutex planningMtx;

    bool isPlanning;


};
} /*namespace wm */
} /*namespace wumpus*/
