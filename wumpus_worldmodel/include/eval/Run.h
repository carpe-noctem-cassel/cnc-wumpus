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
class Run
{
public:
    Run(std::string worldName);

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

private:
    std::string currentPermutation;
    int fieldSize;
    std::map<std::string, eval::SpawnRequestStatus > spawnRequestStatus;

    std::shared_ptr<eval::Result> currentResult;
    int determineFieldSizeFromFilename(const std::string& filename);
    size_t findFieldsizeEndIdx(const std::string& filename);
    bool wroteHeader;


    std::vector<std::shared_ptr<wumpus::model::Field>> agentPositionsFromEncoding() const;
};
}