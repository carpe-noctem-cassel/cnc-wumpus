#pragma once

#include "Experiment.h"
#include "Result.h"
#include "SpawnRequestStatus.h"
#include <utility>
#include <nonstd/optional.hpp>

namespace eval
{
/**
* Experiment Run contains information of the progress of one world
*/
class Experiment;
class Run
{
public:
    Run(std::string worldName, int worldIdx , bool wroteHeader=true);

    std::string worldName;
    eval::CompletionStatus completionStatus;
//    std::map<std::string, eval::Result*> resultsByWorld;
    std::vector<std::shared_ptr<wumpus::model::Field>> getNextStartPositions();
    std::string getCurrentStartPositionsEncoding();
    std::shared_ptr<eval::Result> getCurrentResult();
    bool spawnRequestProcessed(const std::string &encoding);
    bool spawnRequestRegistered(const std::string &encoding);
    void registerSpawnRequest(const std::string& encoding);
    void setSpawnRequestStatus(const std::string& encoding, SpawnRequestStatus status);

    nonstd::optional<SpawnRequestStatus> getSpawnRequestStatus(const std::string& encoding);

    bool isCompleted();
    void saveToDiskAndClearResult();

    static std::mutex runMtx;
private:
    std::string currentPermutation;
    int fieldSize;
    std::map<std::string, eval::SpawnRequestStatus > spawnRequestStatus;

    std::shared_ptr<eval::Result> currentResult;
    int determineFieldSizeFromFilename(const std::string& filename);
    bool determineHasNextPermutation();
    size_t findFieldsizeEndIdx(const std::string& filename);
    bool wroteHeader;

    std::string getNextPermutationVisitAll();

    bool performedLastRun;
    bool hasNextPermutation;
    bool permutateRandomly;

    int encodingsIdx;

    std::set<int> openIndices;

    int runsPerformed;
    int maxRuns;
    std::vector<std::string> encodingsList;

    std::unordered_set<std::string> handledPermutations;
    std::string spawnPlacesFilePath;


    std::vector<std::shared_ptr<wumpus::model::Field>> agentPositionsFromEncoding(std::string encoding) const;

    std::string spawnConfig;

//    std::mutex<
        int getRandomSetElement() const;

    std::string getEncodingBySpawnConfig();
};
}