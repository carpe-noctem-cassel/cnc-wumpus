#include "DomainBehaviour.h"

using std::string;

namespace alica
{
	DomainBehaviour::DomainBehaviour(std::string name) :
			BasicBehaviour(name)
	{
		sc = supplementary::SystemConfig::getInstance();
		ros::NodeHandle n;

		wumpusActionRequestTopic = (*sc)["WumpusWorldModel"]->get < string > ("Send.ActionRequest", NULL);
		wumpusPoseRequestTopic = (*sc)["WumpusWorldModel"]->get < string > ("Send.SpawnAgentRequest", NULL);

		wumpusActionPublisher = n.advertise < wumpus_simulator::ActionRequest > (wumpusActionRequestTopic, 10);
		wumpusPosePublisher = n.advertise < wumpus_simulator::InitialPoseRequest > (wumpusPoseRequestTopic, 10);

	}

	void DomainBehaviour::send(wumpus_simulator::InitialPoseRequest& msg)
	{
		wumpusPosePublisher.publish(msg);
	}

	void DomainBehaviour::send(wumpus_simulator::ActionRequest& msg)
	{
		wumpusActionPublisher.publish(msg);
}

	DomainBehaviour::~DomainBehaviour()
	{
	}
} /* namespace alica */
