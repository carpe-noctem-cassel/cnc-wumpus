#include "eval/Result.h"
#include <FileSystem.h>
#include <SystemConfig.h>
#include <engine/AlicaClock.h>
#include <fstream>
namespace eval
{


Result::Result(int agentCount, bool communicationAllowed)
        : timeMeasurementStart(nonstd::nullopt)
{
    auto sc = essentials::SystemConfig::getInstance();
    this->agentCount = agentCount;
    this->communicationAllowed = communicationAllowed;
    this->resultsDirectory = (*sc)["WumpusEval"]->get<std::string>("TestRun.resultsDirectory", NULL);
    this->alicaClock = new alica::AlicaClock();
}

Result::~Result()
{
    delete this->alicaClock; // TODO access AC through AE through WM singleton?
}

/**
 * Starts duration measurement for one run in one world.
 * Can only be started once.
 */
void Result::startTimeMeasurement()
{
    if (this->timeMeasurementStart) {
        std::cerr << "Result: DurationStart already set!" << std::endl;
        return;
    }
    this->timeMeasurementStart = this->alicaClock->now();
}

/**
 * Stops time measurement and returns the measured time.
 * Time measurement is only allowed to happen once.
 * @return
 */
nonstd::optional<alica::AlicaTime> Result::stopTimeMeasurement()
{
    if (!this->timeMeasurementStart) {
        std::cerr << "Result: Missing start time for duration measurement!" << std::endl;
        return nonstd::nullopt;
    }
    this->duration = this->alicaClock->now() - *this->timeMeasurementStart;
    return this->duration;
}

/**
 * Adds a field to a path associated with an agent.
 * This info can come from perception messages from the simulator of from the knowledge base of the agent, so not
 * thread safe.
 * @param agent
 * @param field
 */
void Result::addToAgentPath(std::shared_ptr<wumpus::model::Agent> agent, std::shared_ptr<wumpus::model::Field> field)
{
    std::lock_guard<std::mutex> lock(this->agentPathMtx);
    if (this->pathByAgent.find(agent) != this->pathByAgent.end()) {
        this->pathByAgent.at(agent).push_back(field);
    } else {
        this->pathByAgent.emplace(agent, std::vector<std::shared_ptr<wumpus::model::Field>>({field}));
    }
}

/**
 * Increases the counter for an agents' action costs or initializes it with 1 if the agent was not yet known.
 */
void Result::increaseActionsCostCounter(int agentId)
{
    if (this->actionsCostByAgent.find(agentId) != this->actionsCostByAgent.end()) {
        this->actionsCostByAgent.at(agentId) = this->actionsCostByAgent.at(agentId) + 1;
    } else {
        this->actionsCostByAgent.emplace(agentId, 1);
    }
}

void Result::serialize(const std::string& fileName)
{
    std::ofstream fileWriter; //TODO extract
    fileWriter.open(essentials::FileSystem::combinePaths(this->resultsDirectory, fileName), std::ios_base::app);
    fileWriter << std::fixed; // << "\t";
    fileWriter << this->worldName;
    fileWriter << "\t" << this->agentCount;
    fileWriter << "\t" << this->communicationAllowed;
    fileWriter << "\t" << this->duration->inSeconds();
    fileWriter << "\t" << this->completionStatus;
    fileWriter << "\t" << this->deadCount;
    fileWriter << "\t" << this->exitedCount;

    //actual result data

    for(const auto& elem : this->actionsCostByAgent) {
        fileWriter << "\t" << elem.second;
    }
    fileWriter << std::endl;
}

void Result::writeHeader(const std::string& fileName) {
    std::ofstream fileWriter;
    fileWriter.open(essentials::FileSystem::combinePaths(this->resultsDirectory, fileName), std::ios_base::app);
    fileWriter << std::fixed; // << "\t";
    //csv header
    fileWriter << "World\tAgentCount\tCommunicationAllowed\tTimeElapsed\tCompletionStatus\tDeadCount\tExitedCount";
    for(const auto& agentIdCostPair : this->actionsCostByAgent) {
        fileWriter << "\tScoreAgent" << agentIdCostPair.first;
    }
    fileWriter << std::endl;
}
}