#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"
#include "wumpus/WumpusWorldModel.h"
#include "wumpus_simulator/ActionRequest.h"
#include "wumpus_simulator/InitialPoseRequest.h"
#include <wumpus_msgs/AgentPerception.h>

namespace essentials
{
	class SystemConfig;
}

namespace alica
{
	class DomainBehaviour : public BasicBehaviour
	{
	public:
		DomainBehaviour(std::string name);
		virtual ~DomainBehaviour();

		void send(wumpus_simulator::InitialPoseRequest& msg);
		void send(wumpus_simulator::ActionRequest& msg);
        void send(wumpus_msgs::AgentPerception& msg);

		wumpus::WumpusWorldModel* wm;

	protected:
		essentials::SystemConfig* sc;

	private:
		std::string wumpusActionRequestTopic;
		std::string wumpusPoseRequestTopic;
        std::string agentPerceptionTopic;

		ros::Publisher wumpusActionPublisher;
		ros::Publisher wumpusPosePublisher;
        ros::Publisher agentPerceptionPublisher;

	};
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

