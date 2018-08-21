#pragma once

#include <engine/AlicaClock.h>
#include <supplementary/InfoBuffer.h>
#include <supplementary/InformationElement.h>
#include <wumpus_simulator/ActionResponse.h>
#include <wumpus_simulator/InitialPoseResponse.h>

#include <vector>

namespace wumpus_simulator {
ROS_DECLARE_MESSAGE(InitialPoseResponse)
ROS_DECLARE_MESSAGE(ActionResponse)
}

namespace alica {
class AlicaTime;
}
namespace wumpus {

class WumpusWorldModel;

namespace wm {
class WumpusSimData {
public:
    WumpusSimData(WumpusWorldModel *wm);
    virtual ~WumpusSimData();

    // methods for processing ROS messages from Wumpus Sim
    void processInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse);
    void processActionResponse(wumpus_simulator::ActionResponsePtr actionResponse);

    // data access through public buffers
    const supplementary::InfoBuffer<wumpus_simulator::InitialPoseResponse> *getInitialPoseResponseBuffer();
    const supplementary::InfoBuffer<wumpus_simulator::ActionResponse> *getActionResponseBuffer();

private:
    WumpusWorldModel *wm;
    alica::AlicaTime maxValidity;

    alica::AlicaTime initialPoseResponseValidityDuration;
    alica::AlicaTime actionResponseValidityDuration;
    supplementary::InfoBuffer<wumpus_simulator::InitialPoseResponse> *initialPoseResponseBuffer;
    supplementary::InfoBuffer<wumpus_simulator::ActionResponse> *actionResponseBuffer;

};
} /* namespace wm */
} /* namespace wumpus */
