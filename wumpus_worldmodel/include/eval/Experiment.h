#pragma once

#include <wumpus/WumpusWorldModel.h>
#include <eval/Run.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace essentials
{
class SystemConfig;
}
namespace eval
{
class Run;
class Experiment
{
public:
    Experiment();
    bool communicationAllowed;
    int agentCount;
    std::string testRunDirectory;
    std::shared_ptr<eval::Run> getCurrentRun();
    std::string advanceWorld();
    bool hasNextWorld();
    int getPresetAgentCount();

private:
    std::vector<std::string> getFilenamesFromDirectory(const std::string& directory);
    int currentWorldIdx;
    std::map<std::string, std::shared_ptr<eval::Run>> worldInfo;
    std::shared_ptr<eval::Run> run;
    std::vector<std::string> worlds;
    wumpus::WumpusWorldModel* wm;
};
}
