#include "wumpus/wm/WumpusSimData.h"

#include "wumpus/WumpusWorldModel.h"

#include <SystemConfig.h>
#include <engine/AlicaClock.h>

#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <utility>

using supplementary::InformationElement;
using supplementary::InfoBuffer;
using std::make_shared;
using std::shared_ptr;

namespace wumpus {
namespace wm {

WumpusSimData::WumpusSimData(WumpusWorldModel *wm) {
    this->wm = wm;
    auto sc = this->wm->getSystemConfig();
    this->maxValidity = alica::AlicaTime::nanoseconds(1000000000);

    //data buffers

    this->initialPoseResponseValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.InitialPoseResponse.ValidityDuration", NULL));
    this->initialPoseResponseBuffer = new InfoBuffer<wumpus_simulator::InitialPoseResponse>((*sc)["WumpusWorldModel"]->get<int>("Data.InitialPoseResponse.BufferLength", NULL));

    this->actionResponseValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.ValidityDuration", NULL));
    this->actionResponseBuffer = new InfoBuffer<wumpus_simulator::ActionResponse>((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.BufferLength", NULL));


}

WumpusSimData::~WumpusSimData() {
}

void WumpusSimData::processInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse)
{
    auto initialPoseResponseInfo = std::make_shared<InformationElement<wumpus_simulator::InitialPoseResponse>>(*(initialPoseResponse.get()), wm->getTime(), this->initialPoseResponseValidityDuration, 1.0);
    initialPoseResponseBuffer->add(initialPoseResponseInfo);
}


void WumpusSimData::processActionResponse(wumpus_simulator::ActionResponsePtr actionResponse)
{
    auto actionResponseInfo = std::make_shared<InformationElement<wumpus_simulator::ActionResponse>>(*(actionResponse.get()), wm->getTime(), this->actionResponseValidityDuration, 1.0);
    actionResponseBuffer->add(actionResponseInfo);
}


const supplementary::InfoBuffer<wumpus_simulator::InitialPoseResponse> *WumpusSimData::getInitialPoseResponseBuffer()
{
	return this->initialPoseResponseBuffer;
}


const supplementary::InfoBuffer<wumpus_simulator::ActionResponse> *WumpusSimData::getActionResponseBuffer()
{
	return this->actionResponseBuffer;
}

}
} /* namespace wumpus */
