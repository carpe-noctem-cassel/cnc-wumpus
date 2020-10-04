using namespace std;
#include "Plans/SingleAgent/Behaviours/SpawnAgent.h"

/*PROTECTED REGION ID(inccpp1534835348868) ENABLED START*/ // Add additional includes here
#include <eval/AgentInfo.h>
#include <wumpus/model/Field.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1534835348868) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
SpawnAgent::SpawnAgent()
        : DomainBehaviour("SpawnAgent")
{
    /*PROTECTED REGION ID(con1534835348868) ENABLED START*/ // Add additional options here
    std::cout << "SpawnAgent Constructor" << std::endl;
    this->nextStartPositions = std::vector<std::shared_ptr<wumpus::model::Field>>(); // wm->experiment->getCurrentRun()->getNextStartPositions(); //already set
                                                                                     // in initialise parameters
    this->loadedInitialWorld = 0;
    this->worldRequestSent = false;
    /*PROTECTED REGION END*/
}
SpawnAgent::~SpawnAgent()
{
    /*PROTECTED REGION ID(dcon1534835348868) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void SpawnAgent::run(void* msg)
{
    /*PROTECTED REGION ID(run1534835348868) ENABLED START*/ // Add additional options here
                                                            /*
                                                             * This Behaviour runs in the initial state as well as when one run in one world has been completed.
                                                             */

    if (this->loadedInitialWorld <= 10) {
        auto request = this->wm->experiment->getCurrentRun()->worldName;
        wumpus::model::communication::LoadWorldRequestData loadWorldRequest;
        loadWorldRequest.worldPath = std::string(getenv("HOME")) + "/" + this->wm->experiment->testRunDirectory + "/" + request;
        send(loadWorldRequest);
        ++loadedInitialWorld;
        //        std::cout << "sending load world request 1 " << std::endl;
        return;
    }
    // agents aren't associated with the playground unless there was info from the simulator that they were recognized ;)
    auto idx = this->wm->playground->getOwnAgentIndex();
    if (!idx) {
        std::cout << "SpawnAgent: missing index!" << std::endl;
        return;
    }
    // only advance next start positions if there was a failure message
    auto registered = this->wm->experiment->getCurrentRun()->spawnRequestRegistered(this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding());
    if (!registered) {
        this->sendMultiInitialPoseRequest();
        this->wm->experiment->getCurrentRun()->registerSpawnRequest(this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding());
        return;
    }

    auto status = this->wm->experiment->getCurrentRun()->getSpawnRequestStatus(this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding());

    if (status == eval::SpawnRequestStatus::FAILED) {
        this->wm->experiment->getCurrentRun()->completionStatus = eval::CompletionStatus::REJECTED;
        // this will advance the starting positions encoding
        std::lock_guard<std::mutex> lock(eval::Experiment::runMtx);
        this->nextStartPositions = wm->experiment->getCurrentRun()->getNextStartPositions();
    }

    // TODO make clearer - empty start positions mean that the  previous world has been completed
    if (this->nextStartPositions.empty() && !worldRequestSent) {
        auto request = this->wm->experiment->advanceWorld();
        wumpus::model::communication::LoadWorldRequestData loadWorldRequest;
        loadWorldRequest.worldPath = std::string(getenv("HOME")) + "/" + this->wm->experiment->testRunDirectory + "/" + request;
        //        std::cout << "Sending load World request2 " << std::endl;
        send(loadWorldRequest);
        worldRequestSent = true;
        std::lock_guard<std::mutex> lock(eval::Experiment::runMtx);
        //        if (wm->getPresetAgentCount() > 1) {
        //            while (wm->experiment->getCurrentRun()->getCurrentResult()->getAgentInfos().size() != wm->getPresetAgentCount()) {
        //                std::cout << "wait for agent infos!" << std::endl;
        //            }
        //        }
        this->nextStartPositions = this->wm->experiment->getCurrentRun()->getNextStartPositions();
        return;
    }

    this->sendMultiInitialPoseRequest();
    this->wm->experiment->getCurrentRun()->registerSpawnRequest(this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding());

    this->awaitingResponse = true;
    /*PROTECTED REGION END*/
}
void SpawnAgent::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1534835348868) ENABLED START*/ // Add additional options here
    std::lock_guard<std::mutex> lock(eval::Experiment::runMtx);

    if (wm->experiment && wm->experiment->getCurrentRun() && wm->experiment->getCurrentRun()->getCurrentResult()) {
        while (wm->experiment->getCurrentRun()->getCurrentResult()->getAgentInfos().size() != wm->getPresetAgentCount()) {
        }
    }

    this->nextStartPositions = wm->experiment->getCurrentRun()->getNextStartPositions();
    this->worldRequestSent = false;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1534835348868) ENABLED START*/ // Add additional methods here
// FIXME move into wumpusSimData


void SpawnAgent::sendMultiInitialPoseRequest()
{

    std::vector<int> agentIds;
    // TODO move to playground as getAgentIDs()
    auto agents = this->wm->getAgentIDsForExperiment();
    for (auto a : agents) {
        agentIds.push_back(a);
    }
    wumpus::model::communication::MultiInitialPoseRequestData multiInitialPoseRequest;
    for (int i = 0; i < this->nextStartPositions.size(); ++i) {
        wumpus::model::communication::AgentPosition agentPosition;
        auto pos = this->nextStartPositions.at(i);
        agentPosition.startX = pos->x;
        agentPosition.startY = pos->y;
        agentPosition.agentId = agentIds.at(i);
        agentPosition.heading = -1;
        multiInitialPoseRequest.agentPositions.push_back(agentPosition);
    }

    //    std::cout << "Sending initial pose request! " << std::endl;
    send(multiInitialPoseRequest);
}
/*PROTECTED REGION END*/
} /* namespace alica */
