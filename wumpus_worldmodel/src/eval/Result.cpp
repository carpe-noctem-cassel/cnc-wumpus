#include "eval/Result.h"
#include <FileSystem.h>
#include <SystemConfig.h>
#include <engine/AlicaClock.h>
#include <fstream>
namespace eval
{

Result::Result(int agentCount, bool communicationAllowed, std::string worldName, std::string encoding)
        : timeMeasurementStart(nonstd::nullopt)
        , worldName(worldName)
        , encoding(encoding)
        , completionStatus(CompletionStatus::UNDEFINED)
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

    std::ofstream fileWriter; // TODO extract and only open writer once
    //fileWriter.open(essentials::FileSystem::combinePaths(this->resultsDirectory, fileName), std::ios_base::app);
    std::cout << "ResultsDirectory: "  << this->resultsDirectory << std::endl;
    auto folder = essentials::FileSystem::combinePaths(getenv("HOME"), this->resultsDirectory);
    fileWriter.open(essentials::FileSystem::combinePaths(folder, fileName), std::ios_base::app);
    std::cout << "writing result to " << essentials::FileSystem::combinePaths(folder, fileName) << std::endl;

    //    fileWriter << std::fixed; // << "\t";
    fileWriter << this->worldName.c_str();
    fileWriter << "\t";
    fileWriter << this->encoding; //.c_str();
    fileWriter << "\t";
    fileWriter << this->agentCount;
    fileWriter << "\t";
    fileWriter << this->communicationAllowed;
    fileWriter << "\t";
    fileWriter << this->duration->inMilliseconds();
    fileWriter << "\t";
    fileWriter << this->completionStatus;
    fileWriter << "\t";
    fileWriter << this->died.size();
    fileWriter << "\t";
    fileWriter << this->exited.size();

    // actual result data

    for (const auto& elem : this->actionsCostByAgent) {
        fileWriter << "\t";
        fileWriter << elem.second;
    }
    fileWriter << std::endl;
    fileWriter.close();
}

void Result::writeHeader(const std::string& fileName)
{
    std::ofstream fileWriter;
    auto folder = essentials::FileSystem::combinePaths(getenv("HOME"), this->resultsDirectory);
    fileWriter.open(essentials::FileSystem::combinePaths(folder, fileName), std::ios_base::app);
    //    fileWriter << std::fixed; // << "\t";
    // csv header
    fileWriter << "World\tEncoding\tAgentCount\tCommunicationAllowed\tTimeElapsed\tCompletionStatus\tDeadCount\tExitedCount";
    for (const auto& agentIdCostPair : this->actionsCostByAgent) {
        fileWriter << "\tScoreAgent";
        fileWriter << agentIdCostPair.first;
    }
    fileWriter << std::endl;
    fileWriter.close();
}
}