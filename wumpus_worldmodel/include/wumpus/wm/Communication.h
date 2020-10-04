#pragma once

#include <engine/AlicaClock.h> /*< needed for AlicaTime */

#include <ros/ros.h>
#include <wumpus_msgs/AgentPerception.h>
#include <wumpus_simulator/ActionRequest.h>
#include <wumpus_simulator/ActionResponse.h>
#include <wumpus_simulator/InitialPoseResponse.h>
#include <wumpus_simulator/MultiInitialPoseResponse.h>
#include <engine/IRoleAssignment.h>
#include <engine/teammanager/TeamManager.h>

namespace wumpus
{
class WumpusWorldModel;
namespace model
{
namespace communication
{
struct InitialPoseRequestData;
struct ActionRequestData;
struct AgentPerceptions;
struct MultiInitialPoseRequestData;
struct LoadWorldRequestData;
}
}
namespace wm
{

class Communication
{
public:
    Communication(wumpus::WumpusWorldModel* wm);
    virtual ~Communication();
    alica::AlicaTime getTimeLastSimMsgReceived();
    void sendTimeoutMessage();
    void sendInitialPoseRequest(wumpus::model::communication::InitialPoseRequestData data);
    void sendActionRequest(wumpus::model::communication::ActionRequestData data);
    void sendAgentPerceptions(wumpus::model::communication::AgentPerceptions data);
    void sendMultiInitialPoseRequest(wumpus::model::communication::MultiInitialPoseRequestData msg);
    void sendLoadWorldRequest(wumpus::model::communication::LoadWorldRequestData data);

//     Allow sending perception messages from WM triggered from plan transitions
    void sendAgentPerception(wumpus_msgs::AgentPerception& msg);

private:
    void onInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse);
    void onActionResponse(wumpus_simulator::ActionResponsePtr actionResponse);
    void onAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception);
    void onMultiInitialPoseResponse(wumpus_simulator::MultiInitialPoseResponsePtr response);

    wumpus::WumpusWorldModel* wm;

    // ROS Stuff
    ros::NodeHandle n;
    ros::AsyncSpinner* spinner;

    ros::Subscriber initialPoseResponseSub;
    ros::Subscriber actionResponseSub;
    ros::Subscriber agentPerceptionSub;
    ros::Subscriber multiAgentSpawnResponseSub;


    // TODO remove this after debugging. timeout should not even happen
    ros::Publisher timeoutPublisher;

    std::string agentPerceptionTopic;

    std::string wumpusActionRequestTopic;
    std::string wumpusPoseRequestTopic;
    std::string multiPoseRequestTopic;
    std::string loadWorldRequestTopic;

    ros::Publisher wumpusActionPublisher;
    ros::Publisher wumpusPosePublisher;
    ros::Publisher agentPerceptionPublisher;
    ros::Publisher multiAgentPoseRequestPublisher;
    ros::Publisher loadWorldRequestPublisher;
};

} /* namespace wm */
} /* namespace wumpus */
