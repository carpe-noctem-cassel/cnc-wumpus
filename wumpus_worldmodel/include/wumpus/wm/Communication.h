#pragma once

#include <engine/AlicaClock.h> /*< needed for AlicaTime */

#include <wumpus_simulator/ActionResponse.h>
#include <wumpus_simulator/InitialPoseResponse.h>
#include <wumpus_msgs/AgentPerception.h>
#include <ros/ros.h>

#define COMM_DEBUG
namespace wumpus
{
class WumpusWorldModel;

namespace wm
{

class Communication
{
  public:
    Communication(wumpus::WumpusWorldModel *wm);
    virtual ~Communication();
    alica::AlicaTime getTimeLastSimMsgReceived();

  private:
    void onInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse);
    void onActionResponse(wumpus_simulator::ActionResponsePtr actionResponse);
    void onAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception);

    wumpus::WumpusWorldModel *wm;

    // ROS Stuff
    ros::NodeHandle n;
    ros::AsyncSpinner *spinner;

    ros::Subscriber initialPoseResponseSub;
    ros::Subscriber actionResponseSub;
    ros::Subscriber agentPerceptionSub;
};

} /* namespace wm */
} /* namespace wumpus */
