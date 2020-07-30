#include "eval/Run.h"
#include <algorithm>
#include <random>
#include <eval/AgentInfo.h>
#include <vector>
#include <wumpus/WumpusWorldModel.h>
#include <wumpus/model/Field.h>
namespace eval
{

static const std::string PG = "pg";
static const std::string PERMUTATE = "permutate";
static const std::string VISIT_ALL_FIELDS = "visit_all";
static const std::string PERMUTATEUNTIL = "permutate_until";
static const std::string EXISTING = "existing";

Run::Run(std::string worldName, bool wroteHeader)
        : completionStatus(CompletionStatus::UNDEFINED)
        , worldName(worldName)
        , currentResult(nullptr)
        , hasNextPermutation(true)
        , performedLastRun(false)
        , runsPerformed(0)
        , encodingsIdx(0)

//        , encodingsList(nonstd::nullopt)
{
    // TODO make fieldSize better accessible
    this->wroteHeader = wroteHeader;
    this->fieldSize = this->determineFieldSizeFromFilename(this->worldName);
    this->currentPermutation = std::string(wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount(), '1');
    this->currentPermutation.resize(this->fieldSize * this->fieldSize, '0');

    this->encodingsList = std::vector<std::string>();

    auto sc = essentials::SystemConfig::getInstance();

    this->spawnConfig = (*essentials::SystemConfig::getInstance())["WumpusEval"]->get<std::string>("TestRun.spawnConfig", NULL);
    this->maxRuns = (*sc)["KnowledgeManager"]->get<int>("nSamples", NULL);
    this->permutateRandomly = (*sc)["WumpusEval"]->get<bool>("TestRun.permutateRandomly", NULL);

    if (this->permutateRandomly) {
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(this->currentPermutation.begin(), this->currentPermutation.end(), g);
    }

    if (this->spawnConfig == VISIT_ALL_FIELDS) {
        for (int i = 0; i < this->fieldSize * this->fieldSize; ++i) {
            this->openIndices.insert(i);
        }
    }
}

/**
* Uses internal string permutation to determine the next spawn positions for all available agents.
 * Should be called after completing a run because results are saved and cleared. TODO move this functionality outside?
* @return next start positions for the current world to distribute agents to. field object may still be empty at this point so return coordinate pair.
*/
std::vector<std::shared_ptr<wumpus::model::Field>> Run::getNextStartPositions()
{
    std::cout << "get next start positions" << std::endl;
    if (this->currentResult) {
//        while (this->currentResult->getAgentInfos().size() != wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount()) {
//            std::cout << "wait for agent infos!" << std::endl;
//        }

        // previous run should be completed
        if (!this->wroteHeader) { // TODO check permutation for initial state instead of flag?
            this->currentResult->writeHeader("Results.csv");
            this->wroteHeader = true;
        }

        if (this->currentResult->completionStatus == CompletionStatus::REJECTED) {
            --runsPerformed;
            this->currentResult->stopTimeMeasurement();
        } else {
            //            std::lock_guard<std::mutex> lock(eval::Experiment::runMtx);
            this->saveToDiskAndClearResult();
        }
    }

    std::cout << "get next start positions 2" << std::endl;

    std::cout << "get next start positions3" << std::endl;

    this->currentPermutation = getEncodingBySpawnConfig();

    //
    //
    //        // this will modify the permutation!
    //        //            this->hasNextPermutation = std::prev_permutation(this->currentPermutation.begin(), this->currentPermutation.end());
    //        while (this->handledPermutations.find(this->currentPermutation) != this->handledPermutations.end()) {
    //            std::random_device rd;
    //            std::mt19937 g(rd());
    //            std::shuffle(this->currentPermutation.begin(), this->currentPermutation.end(), g);
    //            std::cout << "Already handled this permutation! choosing new one " << std::endl;
    //        }

    this->currentResult = std::make_shared<eval::Result>(wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount(),
            wumpus::WumpusWorldModel::getInstance()->experiment->communicationAllowed, this->worldName,
            this->currentPermutation); // TODO consider moving this info

    if (!this->determineHasNextPermutation()) {
        this->performedLastRun = true;
    }

    std::cout << "get next start position4s" << std::endl;

    // a completed run means all spawn positions have been handled
    if (this->isCompleted()) {
        std::cout << "get next start position 5 completed" << std::endl;
        return std::vector<std::shared_ptr<wumpus::model::Field>>();
    }
    std::cout << "get next start positions5" << std::endl;

    std::vector<std::shared_ptr<wumpus::model::Field>> agentPositions = this->agentPositionsFromEncoding(this->currentPermutation);

    auto str = this->currentPermutation;
    this->handledPermutations.insert(str);

    // this will modify the permutation!
    this->hasNextPermutation = this->determineHasNextPermutation();
    if (this->spawnConfig == PERMUTATE && !this->permutateRandomly) {
        std::prev_permutation(this->currentPermutation.begin(), this->currentPermutation.end());
    } else if ((this->spawnConfig == PERMUTATE || this->spawnConfig == PERMUTATEUNTIL) && this->permutateRandomly) {
        while (this->handledPermutations.find(this->currentPermutation) != this->handledPermutations.end()) {
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(this->currentPermutation.begin(), this->currentPermutation.end(), g);
        }
    }

    ++runsPerformed;
    this->currentResult->startTimeMeasurement(); // TODO move to where all agents have been spawned

    return agentPositions;

    //    // a completed run means all spawn positions have been handled
    //    if (this->isCompleted()) {
    //        //    if (runsPerformed > maxRuns) {
    //        return std::vector<std::shared_ptr<wumpus::model::Field>>();
    //    }
    //    std::string encoding;
    //    if (!this->encodingsList.empty()) {
    //        encoding = this->encodingsList.at(encodingsIdx);
    //        if (this->handledPermutations.find(encoding) != this->handledPermutations.end()) {
    //            return std::vector<std::shared_ptr<wumpus::model::Field>>();
    //        } else {
    //
    //            this->handledPermutations.insert(encoding);
    //            this->currentResult = std::make_shared<eval::Result>(wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount(),
    //                    wumpus::WumpusWorldModel::getInstance()->experiment->communicationAllowed, this->worldName, encoding); // TODO consider moving this
    //                    info
    //            encodingsIdx++;
    //        }
    //
    //    } else {
    //
    //        //            this->currentResult = std::make_shared<eval::Result>(wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount(),
    //        //                    wumpus::WumpusWorldModel::getInstance()->experiment->communicationAllowed, this->worldName,
    //        //                    this->currentPermutation); // TODO consider moving this info
    //
    //        auto str = this->currentPermutation;
    //        this->handledPermutations.insert(str);
    //
    //        // this will modify the permutation!
    //        //            this->hasNextPermutation = std::prev_permutation(this->currentPermutation.begin(), this->currentPermutation.end());
    //        while (this->handledPermutations.find(this->currentPermutation) != this->handledPermutations.end()) {
    //            std::random_device rd;
    //            std::mt19937 g(rd());
    //            std::shuffle(this->currentPermutation.begin(), this->currentPermutation.end(), g);
    //            std::cout << "Already handled this permutation! choosing new one " << std::endl;
    //        }
    //        encoding = this->currentPermutation;
    //        this->currentResult = std::make_shared<eval::Result>(wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount(),
    //                wumpus::WumpusWorldModel::getInstance()->experiment->communicationAllowed, this->worldName, encoding); // TODO consider moving this info
    //    }
    //
    //    std::vector<std::shared_ptr<wumpus::model::Field>> agentPositions = this->agentPositionsFromEncoding(encoding);
    //
    //    std::cout << "Return agent positions!" << std::endl;
    //    this->currentResult->startTimeMeasurement(); // TODO move to where all agents have been spawned
    //    ++runsPerformed;
    //    return agentPositions;
}

std::vector<std::shared_ptr<wumpus::model::Field>> Run::agentPositionsFromEncoding(std::string encoding) const
{
    std::vector<std::shared_ptr<wumpus::model::Field>> agentPositions;
    std::pair<int, int> fieldCoordinates;
    std::cout << "current permutation: " << encoding;
    for (int j = 0; j < encoding.size(); ++j) // [0..N-1] integers TODO was only fieldSize before
    {
        if (encoding[j] == '1') {
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
    if (this->spawnConfig == PERMUTATE) {
        return !this->hasNextPermutation && this->performedLastRun;
    }
    if (this->spawnConfig == PERMUTATEUNTIL) {
        return this->runsPerformed >= this->maxRuns;
    }
    if (this->spawnConfig == EXISTING) {
        return this->encodingsIdx == this->encodingsList.size() - 1;
    }
    if (this->spawnConfig == VISIT_ALL_FIELDS) {
        return this->openIndices.empty() && this->performedLastRun;
    }
    std::cout << "RUN: INVALID SPAWN CONFIG" << std::endl;
    throw std::exception();
    return false;
}

std::string Run::getNextPermutationVisitAll()
{
    std::string permutation = std::string(this->fieldSize * this->fieldSize, '0');
    for (int a = 0; a < wumpus::WumpusWorldModel::getInstance()->getPresetAgentCount(); ++a) {
        int i = -1;
        if (this->openIndices.empty()) {
            i = rand() % (fieldSize * fieldSize);
        } else {
            i = getRandomSetElement();
            this->openIndices.erase(i);
        }

        permutation.replace(i, 1, "1");
    }

    return permutation;
}

int Run::getRandomSetElement() const
{
    int idx = rand() % openIndices.size();
    auto it = openIndices.begin();
    for (int i = 0; i < idx; i++) {
        it++;
    }
    return *it;
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

    this->currentResult->stopTimeMeasurement();
    auto filename = "Results.csv";
    //    auto filename = this->worldName + ".csv";
    this->currentResult->serialize(filename);
}

std::shared_ptr<eval::Result> Run::getCurrentResult()
{
    return this->currentResult;
}

bool Run::determineHasNextPermutation()
{
    if (this->spawnConfig == PERMUTATE) { // && !this->permutateRandomly) {
        return this->hasNextPermutation;
    }
    if (this->spawnConfig == PERMUTATEUNTIL) {
        return this->runsPerformed != this->maxRuns;
    }
    if (spawnConfig == EXISTING) {
        return this->encodingsIdx != this->encodingsList.size() - 1;
    }
    if (spawnConfig == VISIT_ALL_FIELDS) {
        return !this->openIndices.empty();
    }
    std::cout << "RUN: INVALID SPAWN CONFIG 2 " << std::endl;
    throw std::exception();
    return false;
}

std::string Run::getEncodingBySpawnConfig()
{
    if (spawnConfig == PERMUTATEUNTIL || spawnConfig == PERMUTATE) {
        if (spawnConfig == PERMUTATEUNTIL && this->runsPerformed == 0 && this->permutateRandomly) {
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(this->currentPermutation.begin(), this->currentPermutation.end(), g);
        }
        return this->currentPermutation;
    }
    if (spawnConfig == VISIT_ALL_FIELDS) {
        return getNextPermutationVisitAll();
    }

    return "";
}
}