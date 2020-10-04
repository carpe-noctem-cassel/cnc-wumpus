#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"
#include "wumpus/WumpusWorldModel.h"
#include <wumpus/model/communication/AgentPerceptions.h>

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

		void send(wumpus::model::communication::InitialPoseRequestData data);
		void send(wumpus::model::communication::ActionRequestData data);
        void send(wumpus::model::communication::AgentPerceptions data);
        void send(wumpus::model::communication::MultiInitialPoseRequestData msg);
        void send(wumpus::model::communication::LoadWorldRequestData data);

		wumpus::WumpusWorldModel* wm;

	protected:
		essentials::SystemConfig* sc;
        wumpus::model::communication::AgentPerceptions createAgentPerception(int ownId) const;


//	private:


    };
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

