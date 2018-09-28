#pragma once

#include "WumpusDefinitions.h"
#include <engine/AlicaClock.h>
#include <supplementary/InfoBuffer.h>
#include <supplementary/InformationElement.h>
#include <wumpus_simulator/ActionResponse.h>
#include <wumpus_simulator/InitialPoseResponse.h>

#include <vector>

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
class Coordinates;
class WumpusSimData
{
  public:
    WumpusSimData(WumpusWorldModel *wm);
    virtual ~WumpusSimData();

    // methods for processing ROS messages from Wumpus Sim
    void processInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse);
    void processActionResponse(wumpus_simulator::ActionResponsePtr actionResponse);

    // data access through public buffers
    const supplementary::InfoBuffer<wumpus_simulator::InitialPoseResponse> *getInitialPoseResponseBuffer();
    const supplementary::InfoBuffer<wumpus_simulator::ActionResponse> *getActionResponseBuffer();
    const supplementary::InfoBuffer<TurnInfo> *getTurnInfoBuffer();

  private:
    WumpusWorldModel *wm;

    alica::AlicaTime initialPoseResponseValidityDuration;
    alica::AlicaTime actionResponseValidityDuration;
    alica::AlicaTime turnInfoValidityDuration;
    supplementary::InfoBuffer<wumpus_simulator::InitialPoseResponse> *initialPoseResponseBuffer;
    supplementary::InfoBuffer<wumpus_simulator::ActionResponse> *actionResponseBuffer;
    supplementary::InfoBuffer<TurnInfo> *turnInfoBuffer;

    bool responsesContain(std::vector<int> responses, int element)
    {
        return (std::find(responses.begin(), responses.end(), element) != responses.end());
    };
};
} /* namespace wm */
} /* namespace wumpus */
