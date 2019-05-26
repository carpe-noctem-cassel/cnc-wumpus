#include "DomainBehaviour.h"

using std::string;

namespace alica
{
	DomainBehaviour::DomainBehaviour(std::string name) :
			BasicBehaviour(name)
	{
		wm = wumpus::WumpusWorldModel::getInstance();
		sc = essentials::SystemConfig::getInstance();
		ros::NodeHandle n;

		wumpusActionRequestTopic = (*sc)["WumpusWorldModel"]->get < string > ("Send.ActionRequest", NULL);
		wumpusPoseRequestTopic = (*sc)["WumpusWorldModel"]->get < string > ("Send.SpawnAgentRequest", NULL);
        agentPerceptionTopic = (*sc)["WumpusWorldModel"]->get < string > ("Send.AgentPerception", NULL);

		wumpusActionPublisher = n.advertise < wumpus_simulator::ActionRequest > (wumpusActionRequestTopic, 10);
		wumpusPosePublisher = n.advertise < wumpus_simulator::InitialPoseRequest > (wumpusPoseRequestTopic, 10);
        agentPerceptionPublisher = n.advertise < wumpus_msgs::AgentPerception > (agentPerceptionTopic, 10);

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

	DomainBehaviour::~DomainBehaviour()
	{
	}
} /* namespace alica */
