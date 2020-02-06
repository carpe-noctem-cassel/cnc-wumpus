#include "DomainBehaviour.h"

using std::string;

namespace alica
{
DomainBehaviour::DomainBehaviour(std::string name)
        : BasicBehaviour(name)
{
    wm = wumpus::WumpusWorldModel::getInstance();
    sc = essentials::SystemConfig::getInstance();
    ros::NodeHandle n;

    wumpusActionRequestTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.ActionRequest", NULL);
    wumpusPoseRequestTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.SpawnAgentRequest", NULL);
    agentPerceptionTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.AgentPerception", NULL);
    multiPoseRequestTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.SpawnMultiAgentRequest", NULL);
    loadWorldRequestTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.LoadWorldRequest", NULL);

    wumpusActionPublisher = n.advertise<wumpus_simulator::ActionRequest>(wumpusActionRequestTopic, 1000);
    wumpusPosePublisher = n.advertise<wumpus_simulator::InitialPoseRequest>(wumpusPoseRequestTopic, 1000);
    agentPerceptionPublisher = n.advertise<wumpus_msgs::AgentPerception>(agentPerceptionTopic, 1000);
    multiAgentPoseRequestPublisher = n.advertise<wumpus_simulator::MultiInitialPoseRequest>(multiPoseRequestTopic, 1000);
    loadWorldRequestPublisher = n.advertise<wumpus_simulator::LoadWorldRequest>(loadWorldRequestTopic, 1000);
}

void DomainBehaviour::send(wumpus_simulator::InitialPoseRequest& msg)
{
    wumpusPosePublisher.publish(msg);
}

void DomainBehaviour::send(wumpus_simulator::ActionRequest& msg)
{
    wumpusActionPublisher.publish(msg);
}
void DomainBehaviour::send(wumpus_msgs::AgentPerception& msg)
{
    agentPerceptionPublisher.publish(msg);
}

void DomainBehaviour::send(wumpus_simulator::MultiInitialPoseRequest& msg)
{
    multiAgentPoseRequestPublisher.publish(msg);
}

void DomainBehaviour::send(wumpus_simulator::LoadWorldRequest& msg)
{
    loadWorldRequestPublisher.publish(msg);
}

DomainBehaviour::~DomainBehaviour()
{
}
} /* namespace alica */
