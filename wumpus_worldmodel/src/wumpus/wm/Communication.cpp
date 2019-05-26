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
    initialPoseResponseSub = n.subscribe(topic, 10, &Communication::onInitialPoseResponse, (Communication*) this);

    topic = (*sc)["WumpusWorldModel"]->get<std::string>("Data.ActionResponse.Topic", NULL);
    actionResponseSub = n.subscribe(topic, 10, &Communication::onActionResponse, (Communication*) this);

    topic = (*sc)["WumpusWorldModel"]->get<std::string>("Data.AgentPerception.Topic", NULL);
    agentPerceptionSub = n.subscribe(topic, 10, &Communication::onAgentPerception, (Communication*) this);

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
} /* namespace wm */
} /* namespace wumpus */
