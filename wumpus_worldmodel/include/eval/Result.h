//#pragma once

//#include <engine/AlicaClock.h>
#include <nonstd/optional.hpp>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <mutex>
#include <engine/AlicaClock.h>
#include "CompletionStatus.h"

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
    void increaseActionsCostCounter(int agentId);
    void serialize(const std::string& fileName);
    void writeHeader(const std::string& fileName);
    void registerExited(int agentId);
    void registerDied(int agentId);

//private: TODO fix accessibility after testing
    nonstd::optional<alica::AlicaTime> timeMeasurementStart;
    nonstd::optional<alica::AlicaTime> duration;
    std::map<std::shared_ptr<wumpus::model::Agent>, std::vector<std::shared_ptr<wumpus::model::Field>>> pathByAgent;
    std::mutex agentPathMtx;
    alica::AlicaClock* alicaClock;
    std::map<int, int> actionsCostByAgent;
    std::string resultsDirectory;
    std::string worldName;
    int agentCount; //TODO cleanup/friend? - some of these fields are duplicate among classes
    bool communicationAllowed;
    eval::CompletionStatus completionStatus;
    std::string encoding;
    std::set<int> exited;
    std::set<int> died;

    //TODO what's with the weird values written to the result file?
    std::mutex resultMtx;

};
}