#pragma once

#include <wumpus/model/Objective.h>
#include <wumpus/model/communication/AgentPerceptions.h>
#include <wumpus_msgs/AgentPerception.h>
#include <wumpus_simulator/ActionRequest.h>
#include <wumpus_simulator/InitialPoseRequest.h>
#include <wumpus_simulator/LoadWorldRequest.h>
#include <wumpus_simulator/MultiInitialPoseRequest.h>
namespace wumpus
{
namespace wm
{
namespace util
{
class ROSCommunicationUtil
{
public:
    static wumpus::model::communication::AgentPerceptions toAgentPerception(wumpus_msgs::AgentPerceptionPtr perception)
    {
        wumpus::model::communication::AgentPerceptions ret;
        ret.senderID = perception->senderID;
        ret.initialPosition = ROSCommunicationUtil::toCoordinates(perception->initialPosition);
        ret.position = ROSCommunicationUtil::toCoordinates(perception->position);
        ret.heading = perception->heading;
        ret.arrow = perception->arrow;
        ret.shot = perception->shot;
        ret.objective = ROSCommunicationUtil::stdStringToObjective(perception->objective);
        for (auto coords : perception->possibleNextFields) {
            ret.possibleNextFields.push_back(ROSCommunicationUtil::toCoordinates(coords));
        }
        for (auto coords : perception->shootingTargets) {
            ret.shootingTargets.push_back(ROSCommunicationUtil::toCoordinates(coords));
        }

        for (auto coords : perception->blockingTraps) {
            ret.blockingTraps.push_back(ROSCommunicationUtil::toCoordinates(coords));
        }

        for (auto coords : perception->blockingWumpi) {
            ret.blockingWumpi.push_back(ROSCommunicationUtil::toCoordinates(coords));
        }

        ret.encoding = perception->encoding;
        ret.died = perception->died;
        ret.diedOn = ROSCommunicationUtil::toCoordinates(perception->diedOn);
        ret.exited = perception->exited;
        ret.receivedScream = perception->receivedScream;
        ret.receivedSilence = perception->receivedSilence;
        ret.exhausted = perception->exhausted;
        ret.haveGold = perception->haveGold;
        ret.stinky = perception->stinky;
        ret.glitter = perception->glitter;
        ret.drafty = perception->drafty;
        ret.haveGold = perception->haveGold;
        return ret;
    }

    static wumpus::model::communication::Coordinates toCoordinates(wumpus_msgs::Coordinates coords)
    {
        wumpus::model::communication::Coordinates ret;
        ret.x = coords.x;
        ret.y = coords.y;
        return ret;
    }

    static wumpus::model::Objective stdStringToObjective(std::string objective)
    {
        auto it = model::objectiveLookup.find(objective);
        if (it != model::objectiveLookup.end()) {
            return it->second;
        }
        std::cout << "[ROSCommunicationUtil.h] Invalid objective " << objective << std::endl;
        throw std::exception();
    }

    static wumpus::model::communication::InitialPosePerception toInitialPosePerception(wumpus_simulator::InitialPoseResponsePtr initialPosePerception)
    {
        wumpus::model::communication::InitialPosePerception ret;
        ret.agentId = initialPosePerception->agentId;
        ret.heading = initialPosePerception->heading;
        ret.x = initialPosePerception->x;
        ret.y = initialPosePerception->y;
        ret.fieldSize = initialPosePerception->fieldSize;
        ret.hasArrow = initialPosePerception->hasArrow;

        return ret;
    }

    static wumpus::model::communication::ActionResponsePerception toActionResponsePerception(wumpus_simulator::ActionResponsePtr actionResponse)
    {
        wumpus::model::communication::ActionResponsePerception ret;
        ret.agentId = actionResponse->agentId;
        ret.x = actionResponse->x;
        ret.y = actionResponse->y;
        ret.heading = actionResponse->heading;
        for (auto resp : actionResponse->responses) {
            ret.responses.push_back(resp);
        }
        return ret;
    }

    static wumpus_simulator::InitialPoseRequest toInitialPoseRequestMsg(model::communication::InitialPoseRequestData data)
    {
        wumpus_simulator::InitialPoseRequest req;
        req.agentId = data.agentId;
        req.startX = data.startX;
        req.startY = data.startY;
        return req;
    }

    static wumpus_simulator::ActionRequest toActionRequestMsg(model::communication::ActionRequestData data)
    {
        wumpus_simulator::ActionRequest req;
        req.agentId = data.agentId;
        req.action = data.action;
        return req;
    }

    static wumpus_msgs::AgentPerception toAgentPerceptionMsg(model::communication::AgentPerceptions perceptions)
    {
        wumpus_msgs::AgentPerception perception;
        perception.senderID = perceptions.senderID;
        perception.initialPosition = ROSCommunicationUtil::toCoordinatesMsg(perceptions.initialPosition);
        perception.position = ROSCommunicationUtil::toCoordinatesMsg(perceptions.position);
        perception.diedOn = ROSCommunicationUtil::toCoordinatesMsg(perceptions.diedOn);
        perception.heading = perceptions.heading;
        perception.haveGold = perceptions.haveGold;
        perception.drafty = perceptions.drafty;
        perception.glitter = perceptions.glitter;
        perception.stinky = perceptions.stinky;
        perception.arrow = perceptions.arrow;
        perception.shot = perceptions.shot;
        for (auto shootingPos : perceptions.shootingTargets) {
            perception.shootingTargets.push_back(ROSCommunicationUtil::toCoordinatesMsg(shootingPos));
        }
        for (auto blockingWumpus : perceptions.shootingTargets) {
            perception.blockingWumpi.push_back(ROSCommunicationUtil::toCoordinatesMsg(blockingWumpus));
        }
        for (auto blockingTrap : perceptions.shootingTargets) {
            perception.blockingTraps.push_back(ROSCommunicationUtil::toCoordinatesMsg(blockingTrap));
        }
        for (auto possibleNext : perceptions.possibleNextFields) {
            perception.possibleNextFields.push_back(ROSCommunicationUtil::toCoordinatesMsg(possibleNext));
        }
        perception.exhausted = perceptions.exhausted;
        perception.exited = perceptions.exited;
        perception.died = perceptions.died;
        std::stringstream obj;
        obj << perceptions.objective;
        perception.objective = obj.str();
        perception.encoding = perceptions.encoding;
        perception.receivedSilence = perceptions.receivedSilence;
        perception.receivedScream = perceptions.receivedScream;

        return perception;
    }

    static wumpus_simulator::MultiInitialPoseRequest toMultiInitialPoseRequestMsg(model::communication::MultiInitialPoseRequestData data)
    {
        wumpus_simulator::MultiInitialPoseRequest req;
        for (auto pos : data.agentPositions) {
            req.requestedPositions.push_back(ROSCommunicationUtil::toAgentPositionMsg(pos));
        }
        return req;
    }

    static wumpus_simulator::LoadWorldRequest toLoadWorldRequestMsg(model::communication::LoadWorldRequestData data)
    {
        wumpus_simulator::LoadWorldRequest req;
        req.worldPath = data.worldPath;
        return req;
    }

    static wumpus_msgs::Coordinates toCoordinatesMsg(model::communication::Coordinates coordinates)
    {
        wumpus_msgs::Coordinates coords;
        coords.x = coordinates.x;
        coords.y = coordinates.y;
        return coords;
    }

    static wumpus_simulator::AgentPosition toAgentPositionMsg(model::communication::AgentPosition pos)
    {
        wumpus_simulator::AgentPosition coords;
        coords.agentId = pos.agentId;
        coords.x = pos.startX;
        coords.y = pos.startY;
        coords.heading = pos.heading;
        return coords;
    }
};
}
}
}