#include "wumpus/wm/Communication.h"

#include "wumpus/WumpusWorldModel.h"

#include <supplementary/AgentID.h>
#include <supplementary/BroadcastID.h>

using std::string;

namespace wumpus {
namespace wm {

Communication::Communication(wumpus::WumpusWorldModel *wm) :
		wm(wm) {
	this->timeLastSimMsgReceived = alica::AlicaTime::zero();
	auto sc = wm->getSystemConfig();
	// SET ROS STUFF
	string topic;

	topic = (*sc)["WumpusWorldModel"]->get < string
			> ("Data.InitialPoseResponse.Topic", NULL);
	initialPoseResponseSub = n.subscribe(topic, 10, &Communication::onInitialPoseResponse,
			(Communication *) this);

	topic = (*sc)["WumpusWorldModel"]->get < string
			> ("Data.ActionResponse.Topic", NULL);
	actionResponseSub = n.subscribe(topic, 10, &Communication::onActionResponse,
			(Communication *) this);

	spinner = new ros::AsyncSpinner(4);
	spinner->start();
}

Communication::~Communication() {
	spinner->stop();
	delete spinner;
}

void Communication::onInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse) {
	this->wm->wumpusSimData.processInitialPoseResponse(initialPoseResponse);
}

void Communication::onActionResponse(wumpus_simulator::ActionResponsePtr actionResponse) {
	this->wm->wumpusSimData.processActionResponse(actionResponse);
}
} /* namespace wm */
} /* namespace wumpus */
