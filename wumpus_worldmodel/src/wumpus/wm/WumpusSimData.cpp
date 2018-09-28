#include "wumpus/wm/WumpusSimData.h"

#include "wumpus/WumpusWorldModel.h"
#include "wumpus/wm/ASPKnowledgeBase.h"
#include "wumpus/wm/Field.h"
#include <WumpusEnums.h>

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

namespace wumpus
{
namespace wm
{

WumpusSimData::WumpusSimData(WumpusWorldModel *wm)
{
    this->wm = wm;
    auto sc = this->wm->getSystemConfig();

    // data buffers

    this->initialPoseResponseValidityDuration =
        alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.InitialPoseResponse.ValidityDuration", NULL));
    this->initialPoseResponseBuffer =
        new InfoBuffer<wumpus_simulator::InitialPoseResponse>((*sc)["WumpusWorldModel"]->get<int>("Data.InitialPoseResponse.BufferLength", NULL));

    this->actionResponseValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.ValidityDuration", NULL));
    this->actionResponseBuffer =
        new InfoBuffer<wumpus_simulator::ActionResponse>((*sc)["WumpusWorldModel"]->get<int>("Data.ActionResponse.BufferLength", NULL));
    this->turnInfoValidityDuration = alica::AlicaTime::nanoseconds((*sc)["WumpusWorldModel"]->get<int>("Data.TurnInfo.ValidityDuration", NULL));
    this->turnInfoBuffer = new InfoBuffer<TurnInfo>((*sc)["WumpusWorldModel"]->get<int>("Data.TurnInfo.BufferLength", NULL));
}

WumpusSimData::~WumpusSimData()
{
}

/**
 * Adds new agent to list of known agents
 * Sets Playground size if necessary
 */
void WumpusSimData::processInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse)
{
    this->wm->playground.initializePlayground(initialPoseResponse->fieldSize);
    auto agent = std::make_shared<wumpus::wm::WumpusAgent>(initialPoseResponse->agentId, initialPoseResponse->x, initialPoseResponse->y,
                                                           initialPoseResponse->heading, initialPoseResponse->hasArrow);
    this->wm->playground.addAgent(agent);
    stringstream ss;
    ss << "fieldSize(" << initialPoseResponse->fieldSize << ").";
    this->wm->knowledgeManager.updateKnowledgeBase(ss.str());
}

/**
 *
 */
void WumpusSimData::processActionResponse(wumpus_simulator::ActionResponsePtr actionResponse)
{

    wumpus::wm::Coordinates coords(actionResponse->x, actionResponse->y);
    auto field = this->wm->playground.getField(coords);
    if(!field) {
    	std::cout << "No field with given coordinates (" << coords.x << ", " << coords.y << ") exists (yet)!" << std::endl;
    	return;
    }
    // TurnInfo
    TurnInfo t;
    t.isMyTurn = false;
    if (actionResponse->agentId == this->wm->getSystemConfig()->getOwnRobotID())
    {

        // check turn info
        if (std::find(actionResponse->responses.begin(), actionResponse->responses.end(), WumpusEnums::yourTurn) != actionResponse->responses.end())
        {
            t.isMyTurn = true;
        }

        auto me = this->wm->playground.getAgentById(this->wm->getSystemConfig()->getOwnRobotID());
        me->currentPosition = field;
        me->currentHeading = actionResponse->heading;
    }
    auto turnInfo = std::make_shared<InformationElement<TurnInfo>>(t, wm->getTime(), this->turnInfoValidityDuration, 1.0);
    this->turnInfoBuffer->add(turnInfo);

    // FieldInfo
    if (responsesContain(actionResponse->responses, WumpusEnums::drafty))
    {
        field->perceptions.push_back("drafty");
    }

    if (responsesContain(actionResponse->responses, WumpusEnums::stinky))
    {
        field->perceptions.push_back("stinky");
    }

    if (responsesContain(actionResponse->responses, WumpusEnums::shiny))
    {
        field->perceptions.push_back("shiny");
    }

    if (responsesContain(actionResponse->responses, WumpusEnums::otherAgent))
    {
        field->perceptions.push_back("otherAgent");
    }



    // update own position TODO track turn number?
    stringstream position;
    //TODO associate with Agent
    position << "on" << "(" << coords.x << ", " << coords.y << ").";
    field->perceptions.push_back(position.str());

    stringstream heading;
    heading << "heading(" << actionResponse->heading << ").";
    field->perceptions.push_back(heading.str());

    this->wm->knowledgeManager.updateKnowledgeBase(field);

    // Global Info

    // TODO
    // Add info Element to raw Buffer

    auto actionResponseInfo = std::make_shared<InformationElement<wumpus_simulator::ActionResponse>>(*(actionResponse.get()), wm->getTime(),
                                                                                                     this->actionResponseValidityDuration, 1.0);

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

const supplementary::InfoBuffer<TurnInfo> *WumpusSimData::getTurnInfoBuffer()
{
    return this->turnInfoBuffer;
}
}
} /* namespace wumpus */
