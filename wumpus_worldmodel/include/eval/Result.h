#pragma once

//#include <engine/AlicaClock.h>
#include "CompletionStatus.h"
#include <engine/AlicaClock.h>
#include <map>
#include <memory>
#include <mutex>
#include <nonstd/optional.hpp>
#include <set>
#include <string>
#include <vector>

namespace wumpus
{
namespace model
{
class Agent;
class Field;
}
}
namespace eval
{
class AgentInfo;
/*
 * Contains information about one experiment run in one world
 */
class Result
{
public:
    Result(int agentCount, bool communicationAllowed, std::string worldName, std::string encoding);
    ~Result();

    void startTimeMeasurement();
    nonstd::optional<alica::AlicaTime> stopTimeMeasurement();
    void addToAgentPath(std::shared_ptr<wumpus::model::Agent> agent, std::shared_ptr<wumpus::model::Field> field);
    std::map<int,AgentInfo> getAgentInfos();
    void updateAgentInfo(int senderId, std::pair<int,int> initialPos, bool died, bool exited);
    void updateAgentInfo(int senderId, bool died, bool exited);
    void increaseActionsCostCounter(int agentId);
    void serialize(const std::string& fileName);
    void writeHeader(const std::string& fileName);
    void registerExited(int agentId);
    void registerDied(int agentId);
    std::map<int, AgentInfo> agentInfos;

    // private: TODO fix accessibility after testing
    nonstd::optional<alica::AlicaTime> timeMeasurementStart;
    nonstd::optional<alica::AlicaTime> duration;
    std::map<std::shared_ptr<wumpus::model::Agent>, std::vector<std::shared_ptr<wumpus::model::Field>>> pathByAgent;
    std::mutex agentPathMtx;
    alica::AlicaClock* alicaClock;
    std::map<int, int> actionsCostByAgent;
    std::string resultsDirectory;
    std::string worldName;
    int agentCount; // TODO cleanup/friend? - some of these fields are duplicate among classes
    bool communicationAllowed;
    eval::CompletionStatus completionStatus;
    const std::string encoding;
    std::set<int> exited;
    std::set<int> died;

    int getNumberOfTrapsFromFilename();
    int getNumberOfWumpiFromFilename();

    // TODO what's with the weird values written to the result file?
    std::mutex resultMtx;
    std::mutex agentInfoMtx;

    int getNumberElementsFilename(char elementCharacter, const char* nextPossibleElements) const;
};
}