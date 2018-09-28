#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"
#include "wumpus/WumpusWorldModel.h"
#include "wumpus_simulator/ActionRequest.h"
#include "wumpus_simulator/InitialPoseRequest.h"

namespace supplementary
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

		wumpus::WumpusWorldModel* wm;

	protected:
		supplementary::SystemConfig* sc;

	private:
		std::string wumpusActionRequestTopic;
		std::string wumpusPoseRequestTopic;

		ros::Publisher wumpusActionPublisher;
		ros::Publisher wumpusPosePublisher;

	};
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

