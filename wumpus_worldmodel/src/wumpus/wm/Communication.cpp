#include "wumpus/wm/Communication.h"

#include "wumpus/WumpusWorldModel.h"

#include <essentials/AgentID.h>
#include <essentials/BroadcastID.h>

namespace wumpus
{
namespace wm
{

Communication::Communication(wumpus::WumpusWorldModel* wm)
        : wm(wm)
{
    auto sc = wm->getSystemConfig();
    // SET ROS STUFF
    std::string topic;

    topic = (*sc)["WumpusWorldModel"]->get<std::string>("Data.InitialPoseResponse.Topic", NULL);
    initialPoseResponseSub = n.subscribe(topic, 1000, &Communication::onInitialPoseResponse, (Communication*) this);

    topic = (*sc)["WumpusWorldModel"]->get<std::string>("Data.ActionResponse.Topic", NULL);
    actionResponseSub = n.subscribe(topic, 1000, &Communication::onActionResponse, (Communication*) this);

    topic = (*sc)["WumpusWorldModel"]->get<std::string>("Data.AgentPerception.Topic", NULL);
    agentPerceptionSub = n.subscribe(topic, 1000, &Communication::onAgentPerception, (Communication*) this);

    ros::NodeHandle n;

    auto wumpusActionRequestTopic = (*sc)["WumpusWorldModel"]->get<std::string>("Send.ActionRequest", NULL);

    this->timeoutPublisher = n.advertise<wumpus_simulator::ActionRequest>(wumpusActionRequestTopic, 100);

    spinner = new ros::AsyncSpinner(4);
    spinner->start();
}

Communication::~Communication()
{
    spinner->stop();
    delete spinner;
}

void Communication::onInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse)
{
    this->wm->wumpusSimData.processInitialPoseResponse(initialPoseResponse);
}

void Communication::onActionResponse(wumpus_simulator::ActionResponsePtr actionResponse)
{
    this->wm->wumpusSimData.processActionResponse(actionResponse);
}

void Communication::onAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception) {
    this->wm->wumpusSimData.processAgentPerception(agentPerception);

}

/**
 * TODO remove after debugging! timeout is not supposed to happen
 */
void Communication::sendTimeoutMessage() {
    std::cout << "IN SEND TIMEOUT MESSAGE" << std::endl;
    wumpus_simulator::ActionRequest msg;
    msg.agentId = essentials::SystemConfig::getOwnRobotID();
    msg.action = WumpusEnums::actions::timeoutRequest;
    this->timeoutPublisher.publish(msg);
}
} /* namespace wm */
} /* namespace wumpus */
