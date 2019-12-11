#include "eval/Experiment.h"
#include <SystemConfig.h>
#include <algorithm>
#include <cstring>
#include <dirent.h>
#include <iostream>
#include <memory>
#include <wumpus/WumpusWorldModel.h>
namespace eval
{

Experiment::Experiment()
{
    auto sc = essentials::SystemConfig::getInstance();
    this->wm = wumpus::WumpusWorldModel::getInstance();
    this->communicationAllowed = (*sc)["WumpusWorldModel"]->get<bool>("Agents.allowCommunication", NULL);
    // TODO keep? this is accessible from WorldModel as well
    this->agentCount = (*sc)["WumpusWorldModel"]->get<int>("Agents.number", NULL);
    this->testRunDirectory = (*sc)["WumpusEval"]->get<std::string>("TestRun.worldsDirectory", NULL);
    this->worlds = getFilenamesFromDirectory(testRunDirectory);
    this->currentWorldIdx = 0;
    for (auto& world : this->worlds) {
        std::cout << "Experiment: " << world << std::endl;
        this->worldInfo.emplace(world, std::make_shared<eval::Run>(world));
    }

    this->run = std::make_shared<eval::Run>(this->worlds.at(currentWorldIdx));
}

/**
 * Advances to the next world to be handled.
 * @return filename(path?) of the next world from the test directory
 */
std::string Experiment::advanceWorld()
{
    std::cout << "Experiment: Advancing world! " << std::endl;

    if (this->currentWorldIdx < this->worlds.size() - 1) {
        this->run->saveToDiskAndClearResult();
        this->run = std::make_shared<eval::Run>(this->worlds.at(++this->currentWorldIdx)); //FIXME cleaner increasing
        return this->worlds.at(this->currentWorldIdx);
    }

    return "";
}

/**
 *
 * @return true if there is another world that has not been solved in the worlds directory
 */
bool Experiment::hasNextWorld()
{
    return this->currentWorldIdx < this->worlds.size() - 1;
}

/**
 * TODO keep this here or in WM only??
 * @return
 */
int Experiment::getPresetAgentCount()
{
    return this->agentCount;
}

/**
 * Provides access to the currently handled world
 * TODO can this always be accessed safely?
 * @return
 */
std::shared_ptr<eval::Run> Experiment::getCurrentRun()
{
    return this->run;
}

/**
 * Extracts filenames from the specified worlds directory
 * @param directory
 * @return
 */
 //TODO use FileSystem utils from essentials
std::vector<std::string> Experiment::getFilenamesFromDirectory(const std::string& directory)
{
    std::vector<std::string> ret;
    auto fullpath = std::string(getenv("HOME")) + "/" + directory;

    DIR* dir;
    struct dirent* entry;
    if ((dir = opendir(fullpath.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((entry = readdir(dir)) != NULL) {
            if (std::strncmp(entry->d_name, ".", 1) != 0 && std::strncmp(entry->d_name, "..", 2) != 0 && entry->d_type != DT_DIR)
                ret.push_back(entry->d_name);
        }
        closedir(dir);
    } else {
        std::cout << "WumpusEval: Error opening directory" << directory << std::endl;
    }
    return ret;
}
}
