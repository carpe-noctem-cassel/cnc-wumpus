#pragma once

#include <WumpusEnums.h>
#include <engine/AlicaClock.h>
#include <supplementary/InfoBuffer.h>
#include <supplementary/InformationElement.h>

#include <map>
#include <mutex>
#include <vector>
#include <algorithm>

#include <chrono>

namespace alica
{
class AlicaTime;
}
namespace wumpus
{

class WumpusWorldModel;
namespace model
{
namespace communication
{
struct AgentPerceptions;
struct InitialPosePerception;
struct ActionResponsePerception;
}
}
namespace wm
{
class WumpusSimData
{
public:
    WumpusSimData(WumpusWorldModel* wm);
    virtual ~WumpusSimData();

    // methods for processing ROS messages from Wumpus Sim
    void processInitialPoseResponse(wumpus::model::communication::InitialPosePerception initialPoseResponse);
    void processActionResponse(wumpus::model::communication::ActionResponsePerception actionResponse);
    void processAgentPerception(wumpus::model::communication::AgentPerceptions agentPerception);

    // data access through public buffers
    const supplementary::InfoBuffer<wumpus::model::communication::ActionResponsePerception>* getActionResponseBuffer();
    // for Transition evaluation in Plans
    const supplementary::InfoBuffer<bool>* getIsMyTurnBuffer();
    void setIntegratedFromSimulator(bool integrated);
    bool getIntegratedFromSimulator();

    void resetIntegratedFromAgents();
    bool isIntegratedFromAllAgents();

    int timesCompletionStatusApplied;

    // should be called when action was performed
    void raiseTurnCounter();
    std::map<int, bool> integratedFromOtherAgentsForTurnNr; // FIXME

    // needed for evaluation (restarting of Base)
    void clearBuffers();

    void incrementAwaitingScreamOrSilence();

    void decrementAwaitingScreamOrSilence();

    bool completionStatusApplied();

    int getAwaitingScreamOrSilence();

    bool communicationAllowed;

    bool getIsAwaitingShootingFeedback() const;

    void setIsAwaitingShootingFeedback(bool isAwaitingShootingFeedback);

private:
    WumpusWorldModel* wm;

    alica::AlicaTime actionResponseValidityDuration;
    alica::AlicaTime turnInfoValidityDuration;

    supplementary::InfoBuffer<wumpus::model::communication::ActionResponsePerception>* actionResponseBuffer;
    supplementary::InfoBuffer<bool>* isMyTurnBuffer;

    int turn;

    std::mutex mtx;
    std::mutex respMtx;
    std::mutex awaitingMtx;
    bool integratedFromSimulatorForTurnNumber;
    int awaitingScreamOrSilence;
    bool isAwaitingShootingFeedback;

    bool responsesContain(std::vector<int>& responses, int element) { return (std::find(responses.begin(), responses.end(), element) != responses.end()); };
};
} /* namespace wm */
} /* namespace wumpus */
