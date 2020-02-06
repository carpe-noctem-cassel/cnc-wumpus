#include "eval/Run.h"
#include <vector>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/model/Field.h>

namespace eval
{

static const std::string PG = "pg";

Run::Run(std::string worldName)
        : completionStatus(CompletionStatus::UNDEFINED)
        , worldName(worldName)
        , currentResult(nullptr)
        , wroteHeader(false)
{
    // TODO make fieldSize better accessible
    this->fieldSize = this->determineFieldSizeFromFilename(this->worldName);
    this->currentPermutation = std::string(wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount(), '1');
    this->currentPermutation.resize(this->fieldSize * this->fieldSize, '0');
}

/**
* Uses internal string permutation to determine the next spawn positions for all available agents.
 * Should be called after completing a run because results are saved and cleared. TODO move this functionality outside?
* @return next start positions for the current world to distribute agents to. field object may still be empty at this point so return coordinate pair.
*/
std::vector<std::shared_ptr<wumpus::model::Field>> Run::getNextStartPositions()
{

    if (this->currentResult) {

        // previous run should be completed
        if (!this->wroteHeader) { // TODO check permutation for initial state instead of flag?
            this->currentResult->writeHeader(this->worldName + ".csv");
            this->wroteHeader = true;
        }

        this->saveToDiskAndClearResult();
    }

    this->currentResult = std::make_shared<eval::Result>(wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount(),
            wumpus::WumpusWorldModel::getInstance()->experiment->communicationAllowed, this->worldName,
            this->currentPermutation); // TODO consider moving this info

    // a completed run means all spawn positions have been handled
    if (this->isCompleted()) {
        return std::vector<std::shared_ptr<wumpus::model::Field>>();
    }

    std::vector<std::shared_ptr<wumpus::model::Field>> agentPositions = this->agentPositionsFromEncoding();

    // this will modify the permutation!
    std::prev_permutation(this->currentPermutation.begin(), this->currentPermutation.end());

    this->currentResult->startTimeMeasurement(); // TODO move to where all agents have been spawned

    return agentPositions;
}

std::vector<std::shared_ptr<wumpus::model::Field>> Run::agentPositionsFromEncoding() const
{
    std::vector<std::shared_ptr<wumpus::model::Field>> agentPositions;
    std::pair<int, int> fieldCoordinates;

    for (int j = 0; j < currentPermutation.size(); ++j) // [0..N-1] integers TODO was only fieldSize before
    {
        if (currentPermutation[j] == '1') {
            fieldCoordinates.first = j % fieldSize; // TODO was only fieldSize before
            fieldCoordinates.second = j / fieldSize;
            auto field = wumpus::WumpusWorldModel::getInstance()->playground->getField(fieldCoordinates.first, fieldCoordinates.second);
            agentPositions.push_back(wumpus::WumpusWorldModel::getInstance()->playground->getField(fieldCoordinates.first, fieldCoordinates.second));
        }
    }
    return agentPositions;
}

/**
 * A single run is completed if a world has been tested with all possible spawn position combinations.
 * @return
 */
bool Run::isCompleted()
{
    auto permutationCpy = this->currentPermutation;
    return !std::prev_permutation(permutationCpy.begin(), permutationCpy.end());
}

/**
 * The wwf_generator automatically encodes the field size in the filename.
 * The fieldSize needs to be known early on, so in order to reduce configuration/communication efforts, this method extracts
 * the value from the filename.
 * @param filename
 * @return
 */
int Run::determineFieldSizeFromFilename(const std::string& filename)
{
    auto startIdx = filename.find(eval::PG) + eval::PG.size();
    auto endIdx = this->findFieldsizeEndIdx(filename);
    return std::stoi(filename.substr(startIdx, endIdx));
}

/**
 * Helper method to determine the end index of the world size.
 */
size_t Run::findFieldsizeEndIdx(const std::string& filename)
{
    auto idx = filename.find('w');
    if (idx != std::string::npos) {
        return idx;
    }
    idx = filename.find('t');
    if (idx != std::string::npos) {
        return idx;
    }
    idx = filename.find('g');
    if (idx != std::string::npos) {
        return idx;
    }
    std::cout << "Run: Could not determine FieldSize from Filename!" << std::endl;
    return std::string::npos;
}

/**
 * @return True, if the spawn request for this encoding has been registered and is not pending.
 */
bool Run::spawnRequestProcessed(const std::string& encoding)
{
    if (this->spawnRequestStatus.find(encoding) != this->spawnRequestStatus.end()) {
        return false;
    }

    return this->spawnRequestStatus.at(encoding) == SpawnRequestStatus::ACCEPTED || this->spawnRequestStatus.at(encoding) == SpawnRequestStatus::FAILED;
}

/**
 * Adds a new spawn request with status 'pending'.
 */
void Run::registerSpawnRequest(const std::string& encoding)
{
    this->spawnRequestStatus.emplace(encoding, SpawnRequestStatus::PENDING);
}

/**
 * Updates information on a spawn request, if it was registered.
 */
void Run::setSpawnRequestStatus(const std::string& encoding, SpawnRequestStatus newStatus)
{
    if (this->spawnRequestStatus.find(encoding) != this->spawnRequestStatus.end()) {
        this->spawnRequestStatus.at(encoding) = newStatus;
    }
    // TODO emplace/error here?
}

/**
 * @return The current encoding of spawn positions.
 */
std::string Run::getCurrentStartPositionsEncoding()
{
    return this->currentPermutation;
}

/**
 * @return The spawn request status, if present, else nullopt.
 */
nonstd::optional<SpawnRequestStatus> Run::getSpawnRequestStatus(const std::string& encoding)
{
    if (this->spawnRequestStatus.find(encoding) != this->spawnRequestStatus.end()) {
        return this->spawnRequestStatus.at(encoding);
    }
    return nonstd::nullopt;
}

/**
 * @return True, if the spawn request was registered.
 */
bool Run::spawnRequestRegistered(const std::string& encoding)
{
    return this->spawnRequestStatus.find(encoding) != this->spawnRequestStatus.end();
}

/**
 * doesn't actually reset result - rename
 * TODO serialize result when
 */
void Run::saveToDiskAndClearResult()
{
    this->currentResult->stopTimeMeasurement(); // TODO move into serialize?
    auto filename = this->worldName + ".csv";
    this->currentResult->serialize(filename);
}

std::shared_ptr<eval::Result> Run::getCurrentResult()
{
    return this->currentResult;
}
}