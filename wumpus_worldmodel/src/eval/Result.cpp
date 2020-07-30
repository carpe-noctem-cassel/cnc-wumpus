#include "eval/Result.h"
#include "eval/AgentInfo.h"
#include <FileSystem.h>
#include <SystemConfig.h>
#include <engine/AlicaClock.h>
#include <fstream>
#include <map>
#include <utility>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/model/Playground.h>

namespace eval
{

Result::Result(int agentCount, bool communicationAllowed, std::string worldName, std::string encoding)
        : timeMeasurementStart(nonstd::nullopt)
        , worldName(std::move(worldName))
        , encoding(std::move(encoding))
        , completionStatus(CompletionStatus::UNDEFINED)
        , agentCount(agentCount)
        , communicationAllowed(communicationAllowed)
{
    auto sc = essentials::SystemConfig::getInstance();
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
    this->timeMeasurementStart = nonstd::nullopt;
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
    if (this->completionStatus == CompletionStatus::REJECTED) {
        return;
    }

    //    "World,Encoding,AgentCount,CommunicationAllowed,AgentsHaveArrows,WumpusCount,
    //    TrapCount,TimeElapsed,CompletionStatus,DeadCount,ExitedCount,Cost123,CompletionResult123,StartPos123";
    auto wm = wumpus::WumpusWorldModel::getInstance();
    std::ofstream fileWriter; // TODO extract and only open writer once
    // fileWriter.open(essentials::FileSystem::combinePaths(this->resultsDirectory, fileName), std::ios_base::app);
    std::cout << "ResultsDirectory: " << this->resultsDirectory << std::endl;
    auto folder = essentials::FileSystem::combinePaths(getenv("HOME"), this->resultsDirectory);
    fileWriter.open(essentials::FileSystem::combinePaths(folder, fileName), std::ios_base::app);
    std::cout << "writing result to " << essentials::FileSystem::combinePaths(folder, fileName) << std::endl;
    auto separator = ";";
    //    fileWriter << std::fixed; // << separator;
    fileWriter << this->worldName.c_str();
    fileWriter << separator;
    fileWriter << wumpus::WumpusWorldModel::getInstance()->playground->getPlaygroundSize(); //.c_str();
    fileWriter << separator;
    fileWriter << this->agentCount;
    fileWriter << separator;
    fileWriter << this->communicationAllowed;
    fileWriter << separator;
    fileWriter << "1"; // agents always have arrows currently
    fileWriter << separator;
    fileWriter << this->getNumberOfWumpiFromFilename();
    fileWriter << separator;
    fileWriter << this->getNumberOfTrapsFromFilename();
    fileWriter << separator;
    fileWriter << this->duration->inMilliseconds();
    fileWriter << separator;
    fileWriter << this->completionStatus;
    fileWriter << separator;
    fileWriter << std::to_string(this->died.size());
    fileWriter << separator;
    fileWriter << std::to_string(this->exited.size());

    // actual result data

    //    for (const auto& elem : this->actionsCostByAgent) {
    for (auto i = 1; i <= 5; ++i) {
        fileWriter << separator;
        if (this->actionsCostByAgent.find(i) != this->actionsCostByAgent.end()) {
            auto elem = this->actionsCostByAgent.at(i);
            if(this->getAgentInfos().at(i).died) {
                elem += 1000;
            }
            fileWriter << std::to_string(elem);
        }
    }

    auto agentInf = this->getAgentInfos();

    //    for (const auto& elem : this->agentInfos) {
    for (auto i = 1; i <= 5; ++i) {
        fileWriter << separator;
        // completion result
        if (agentInf.find(i) != agentInf.end()) {
            auto elem = this->agentInfos.at(i);
            if ((elem.exited && elem.died) || (!elem.died && !elem.exited)) {
                            std::cout << "Result: invalid agent info" << std::endl;
                            throw std::exception();
            }
            fileWriter << (elem.exited ? "EXITED" : "DIED");
        }
    }
    //    for (const auto& elem : this->agentInfos) {
    for (auto i = 1; i <= 5; ++i) {
        fileWriter << separator;
        if (agentInf.find(i) != agentInf.end()) {
            auto agent = this->getAgentInfos().at(i);
            fileWriter << "(";
            fileWriter << std::to_string(agent.initialPos.first);
            fileWriter << "-";
            fileWriter << std::to_string(agent.initialPos.second);
            fileWriter << ")";
        }
    }
    fileWriter << std::endl;
    wumpus::model::Playground::agentsForExperiment.clear();
    std::cout << "wrote result" << std::endl;
    fileWriter.close();
}

void Result::writeHeader(const std::string& fileName)
{
    std::ofstream fileWriter;
    auto folder = essentials::FileSystem::combinePaths(getenv("HOME"), this->resultsDirectory);
    fileWriter.open(essentials::FileSystem::combinePaths(folder, fileName), std::ios_base::app);
    //    fileWriter << std::fixed; // << separator;
    // csv header
    fileWriter << "World;WorldSize;AgentCount;CommunicationAllowed;AgentsHaveArrows;WumpusCount;TrapCount;TimeElapsed;CompletionStatus;DeadCount;ExitedCount";
    //    for (auto id : wumpus::WumpusWorldModel::getInstance()->getAgentIDsForExperiment()) {
    for (auto id = 1; id <= 5; ++id) {
        fileWriter << ";CostsAgent";
        fileWriter << std::to_string(id);
    }
    //    for (auto id : wumpus::WumpusWorldModel::getInstance()->getAgentIDsForExperiment()) {
    for (auto id = 1; id <= 5; ++id) {
        fileWriter << ";CompletionResultAgent";
        fileWriter << std::to_string(id);
    }
    for (auto id = 1; id <= 5; ++id) {
        //    for (auto id : wumpus::WumpusWorldModel::getInstance()->getAgentIDsForExperiment()) {
        fileWriter << ";StartPointAgent";
        fileWriter << std::to_string(id);
    }

    fileWriter << std::endl;
    fileWriter.close();
}

int Result::getNumberOfTrapsFromFilename()
{
    auto elementCharacter = 't';
    auto nextPossibleElements = "wg.";
    return getNumberElementsFilename(elementCharacter, nextPossibleElements);
}

int Result::getNumberOfWumpiFromFilename()
{
    auto elementCharacter = 'w';
    auto nextPossibleElements = "tg.";
    return getNumberElementsFilename(elementCharacter, nextPossibleElements);
}

int Result::getNumberElementsFilename(char elementCharacter, const char* nextPossibleElements) const
{
    auto elementIdxStart = worldName.find(elementCharacter);
    if (elementIdxStart == std::string::npos) {
        return 0;
    }
    auto elementsIdxEnd = worldName.find_first_of(nextPossibleElements, elementIdxStart + 1);
    auto substr = worldName.substr(elementIdxStart + 1, elementsIdxEnd - elementIdxStart);
    std::cout << "SUBSTR IS: " << substr << std::endl;
    return std::count(substr.begin(), substr.end(), '-');
}

std::map<int, eval::AgentInfo> Result::getAgentInfos()
{
    std::lock_guard<std::mutex> lock(this->agentInfoMtx);
    return this->agentInfos;
}

void Result::updateAgentInfo(int senderId, std::pair<int, int> initialPos, bool died, bool exited)
{
    std::lock_guard<std::mutex> lock(this->agentInfoMtx);
    if (this->agentInfos.find(senderId) != this->agentInfos.end()) {
        this->agentInfos.at(senderId).initialPos = initialPos;
        this->agentInfos.at(senderId).exited = exited;
        this->agentInfos.at(senderId).died = died;
    } else {
        this->agentInfos.emplace(senderId, AgentInfo(senderId, initialPos, died, exited));
        return;
    }


}

void Result::updateAgentInfo(int senderId, bool died, bool exited)
{

    std::lock_guard<std::mutex> lock(this->agentInfoMtx);
    if (this->agentInfos.find(senderId) != this->agentInfos.end()) {
        this->agentInfos.at(senderId).exited = exited;
        this->agentInfos.at(senderId).died = died;
    }

}
}