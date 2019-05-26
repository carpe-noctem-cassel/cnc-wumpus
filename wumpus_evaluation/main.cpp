#include <EvalBase.h>
#include <SystemConfig.h>
#include <WumpusSimulatorHeadless.h>
#include <dirent.h>
#include <engine/PlanBase.h>
#include <model/Agent.h>
#include <model/Model.h>

std::vector<std::string> getFilenamesFromDirectory(std::string directory)
{
    std::vector<std::string> ret;
    auto path2home = getenv("HOME");
    auto fullpath = std::string(path2home) + "/" + directory;

    DIR* dir;
    struct dirent* entry;
    if ((dir = opendir(fullpath.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((entry = readdir(dir)) != NULL) {
            if (strncmp(entry->d_name, ".", 1) != 0 && strncmp(entry->d_name, "..", 2) != 0)
                ret.push_back(entry->d_name);
        }
        closedir(dir);
    } else {
        std::cout << "WumpusEval: Error opening directory" << directory << std::endl;
    }
    return ret;
}

int main(int argc, char* argv[])
{
    std::vector<std::string> success;
    std::vector<std::string> failure;
    auto sc = essentials::SystemConfig::getInstance();
    auto testRunDirectory = (*sc)["WumpusEval"]->get<string>("TestRun.worldsDirectory", NULL);
    auto path2home = getenv("HOME");
    auto worlds = getFilenamesFromDirectory(testRunDirectory);

    // needed for node in wumpus simulator model
    ros::init(argc, argv, sc->getHostname() + "_wumpuseval");

    std::cout << "WumpusEvaluation starting with " << worlds.size() << " worlds" << std::endl;
    wumpus_simulator::headless::WumpusSimulatorHeadless sim;


    for (int i = 0; i < worlds.size(); ++i) {
        auto file = worlds.at(i);
        std::cerr << "Loading world " << file << std::endl;
        sim.loadWorld(std::string(path2home) + "/" + testRunDirectory + "/" + file);

        auto base = new wumpus::EvalBase("", "WumpusMaster", "");

        base->start();
        auto start = ros::Time::now();
        while(!base->wm->localAgentDied && !base->wm->localAgentExited) {

        }
        if (base->wm->localAgentExited) {
            success.push_back(file);

        } else if (base->wm->localAgentDied) {
            failure.push_back(file);
        }
        delete base;
        sleep(1);
    }

    std::cout << "Evaluation done!" << std::endl;
    std::cout << "Successful worlds: " << success.size() << "/" << worlds.size() << std::endl;
    std::cout << "Failed worlds: " << failure.size() << "/" << worlds.size() << std::endl;
    if (!failure.empty()) {
        std::cout << "Failed worlds are: " << std::endl;
        for (auto f : failure) {
            std::cout << f << std::endl;
        }
    }

    // delete sim;
}
