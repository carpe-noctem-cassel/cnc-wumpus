#pragma once

#include <engine/AlicaClock.h> /*< needed for AlicaTime */

#include <ros/ros.h>
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
    Communication(wumpus::WumpusWorldModel *wm);
    virtual ~Communication();
    alica::AlicaTime getTimeLastSimMsgReceived();

  private:
    void onInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse);
    void onActionResponse(wumpus_simulator::ActionResponsePtr actionResponse);

    wumpus::WumpusWorldModel *wm;

    // ROS Stuff
    ros::NodeHandle n;
    ros::AsyncSpinner *spinner;

    alica::AlicaTime timeLastSimMsgReceived;

    ros::Subscriber initialPoseResponseSub;
    ros::Subscriber actionResponseSub;
};

} /* namespace wm */
} /* namespace wumpus */
