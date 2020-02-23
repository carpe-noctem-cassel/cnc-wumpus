using namespace std;
#include "Plans/SingleAgent/Behaviours/SpawnAgent.h"

/*PROTECTED REGION ID(inccpp1534835348868) ENABLED START*/ // Add additional includes here
#include <wumpus/model/Field.h>
#include <wumpus_simulator/AgentPosition.h>
#include <wumpus_simulator/InitialPoseRequest.h>
#include <wumpus_simulator/LoadWorldRequest.h>
#include <wumpus_simulator/MultiInitialPoseRequest.h>
#include <engine/IRoleAssignment.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1534835348868) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
SpawnAgent::SpawnAgent()
        : DomainBehaviour("SpawnAgent")
{
    /*PROTECTED REGION ID(con1534835348868) ENABLED START*/ // Add additional options here
    this->multiAgentSpawnResponseSub = n.subscribe("wumpus_simulator/SpawnMultiAgentResponse", 10, &SpawnAgent::onMultiInitialPoseResponse, (SpawnAgent*) this);
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
        wumpus_simulator::LoadWorldRequest loadWorldRequest;
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
        this->nextStartPositions = wm->experiment->getCurrentRun()->getNextStartPositions();
    }

    // TODO make clearer - empty start positions mean that the  previous world has been completed
    if (this->nextStartPositions.empty() && !worldRequestSent) {
        auto request = this->wm->experiment->advanceWorld();
        wumpus_simulator::LoadWorldRequest loadWorldRequest;
        loadWorldRequest.worldPath = std::string(getenv("HOME")) + "/" + this->wm->experiment->testRunDirectory + "/" + request;
        //        std::cout << "Sending load World request2 " << std::endl;
        send(loadWorldRequest);
        worldRequestSent = true;
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
    this->nextStartPositions = wm->experiment->getCurrentRun()->getNextStartPositions();
    this->worldRequestSent = false;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1534835348868) ENABLED START*/ // Add additional methods here
// FIXME move into wumpusSimData
void SpawnAgent::onMultiInitialPoseResponse(wumpus_simulator::MultiInitialPoseResponsePtr response)
{
    if (!response->success) {
        std::cout << "Pose request failed!" << std::endl;
        auto id = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
        if(this->wm->getEngine()->getRoleAssignment()->getRole(id)->getName() == this->wm->spawnRequestHandlerRoleName) {
        this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus = eval::CompletionStatus::REJECTED;
        }

        this->wm->experiment->getCurrentRun()->setSpawnRequestStatus(
                this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding(), eval::SpawnRequestStatus::FAILED);
    } else {
        this->wm->experiment->getCurrentRun()->setSpawnRequestStatus(
                this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding(), eval::SpawnRequestStatus::ACCEPTED);
    }
}

void SpawnAgent::sendMultiInitialPoseRequest()
{

    std::vector<int> agentIds;
    // TODO move to playground as getAgentIDs()
    auto agents = this->wm->getAgentIDsForExperiment();
    for (auto a : agents) {
        agentIds.push_back(a);
    }
    wumpus_simulator::MultiInitialPoseRequest multiInitialPoseRequest;
    for (int i = 0; i < this->nextStartPositions.size(); ++i) {
        wumpus_simulator::AgentPosition agentPosition;
        auto pos = this->nextStartPositions.at(i);
        agentPosition.x = pos->x;
        agentPosition.y = pos->y;
        agentPosition.agentId = agentIds.at(i);
        agentPosition.heading = -1;
        multiInitialPoseRequest.requestedPositions.push_back(agentPosition);
    }

    //    std::cout << "Sending initial pose request! " << std::endl;
    send(multiInitialPoseRequest);
}
/*PROTECTED REGION END*/
} /* namespace alica */
