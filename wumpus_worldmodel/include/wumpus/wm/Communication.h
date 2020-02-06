#pragma once

#include <engine/AlicaClock.h> /*< needed for AlicaTime */

#include <ros/ros.h>
#include <wumpus_msgs/AgentPerception.h>
#include <wumpus_simulator/ActionRequest.h>
#include <wumpus_simulator/ActionResponse.h>
#include <wumpus_simulator/InitialPoseResponse.h>

#define COMM_DEBUG
namespace wumpus
{
class WumpusWorldModel;

namespace wm
{

class Communication
{
public:
    Communication(wumpus::WumpusWorldModel* wm);
    virtual ~Communication();
    alica::AlicaTime getTimeLastSimMsgReceived();
    void sendTimeoutMessage();

private:
    void onInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse);
    void onActionResponse(wumpus_simulator::ActionResponsePtr actionResponse);
    void onAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception);

    wumpus::WumpusWorldModel* wm;

    // ROS Stuff
    ros::NodeHandle n;
    ros::AsyncSpinner* spinner;

    ros::Subscriber initialPoseResponseSub;
    ros::Subscriber actionResponseSub;
    ros::Subscriber agentPerceptionSub;

    // TODO remove this after debugging. timeout should not even happen
    ros::Publisher timeoutPublisher;
};

} /* namespace wm */
} /* namespace wumpus */
