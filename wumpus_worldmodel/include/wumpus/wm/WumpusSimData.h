#pragma once

#include <WumpusEnums.h>
#include <engine/AlicaClock.h>
#include <supplementary/InfoBuffer.h>
#include <supplementary/InformationElement.h>
#include <wumpus_simulator/ActionResponse.h>
#include <wumpus_simulator/InitialPoseResponse.h>

#include <mutex>
#include <vector>
#include <wumpus_msgs/AgentPerception.h>

#include <chrono>

namespace wumpus_msgs
{
ROS_DECLARE_MESSAGE(AgentPerception)
ROS_DECLARE_MESSAGE(Coordinates)
}
namespace wumpus_simulator
{
ROS_DECLARE_MESSAGE(InitialPoseResponse)
ROS_DECLARE_MESSAGE(ActionResponse)
}

namespace alica
{
class AlicaTime;
}
namespace wumpus
{

class WumpusWorldModel;

namespace wm
{
class WumpusSimData
{
public:
    WumpusSimData(WumpusWorldModel* wm);
    virtual ~WumpusSimData();

    // methods for processing ROS messages from Wumpus Sim
    void processInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse);
    void processActionResponse(wumpus_simulator::ActionResponsePtr actionResponse);
    void processAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception);

    // data access through public buffers
    const supplementary::InfoBuffer<wumpus_simulator::ActionResponse>* getActionResponseBuffer();
    // for Transition evaluation in Plans
    const supplementary::InfoBuffer<bool>* getIsMyTurnBuffer();
    void setIntegratedFromSimulator(bool integrated);
    bool getIntegratedFromSimulator();

    void resetIntegratedFromAgents();
    bool isIntegratedFromAllAgents();

    //should be called when action was performed
    void raiseTurnCounter();
    std::map<int, bool> integratedFromOtherAgentsForTurnNr; // FIXME

    // needed for evaluation (restarting of Base)
    void clearBuffers();

    void setAwaitingScreamOrSilence(bool b);

    bool getAwaitingScreamOrSilence();

private:
    WumpusWorldModel* wm;

    alica::AlicaTime actionResponseValidityDuration;
    alica::AlicaTime turnInfoValidityDuration;

    supplementary::InfoBuffer<wumpus_simulator::ActionResponse>* actionResponseBuffer;
    supplementary::InfoBuffer<bool>* isMyTurnBuffer;

    int turn;

    std::mutex mtx;
    std::mutex respMtx;
    std::mutex awaitingMtx;
    bool integratedFromSimulatorForTurnNumber;
    bool awaitingScreamOrSilence;


    bool responsesContain(std::vector<int>& responses, int element) { return (std::find(responses.begin(), responses.end(), element) != responses.end()); };


};
} /* namespace wm */
} /* namespace wumpus */
