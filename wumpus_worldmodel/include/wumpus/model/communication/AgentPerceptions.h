#pragma once

#include "wumpus/model/Objective.h"
#include <string>
namespace wumpus
{
namespace model
{
namespace communication
{

struct Coordinates
{
    int x;
    int y;
};
struct AgentPerceptions
{
    int senderID;
    std::string encoding;
    Coordinates initialPosition;
    Coordinates position;
    Coordinates diedOn;
    std::vector<Coordinates> blockingWumpi;
    std::vector<Coordinates> blockingTraps;
    std::vector<Coordinates> possibleNextFields;
    std::vector<Coordinates> shootingTargets;
    int heading;
    bool haveGold;
    bool drafty;
    bool stinky;
    bool glitter;
    bool arrow;
    bool exhausted;
    bool shot;
    bool receivedScream;
    bool receivedSilence;
    bool exited;
    bool died;
    wumpus::model::Objective objective;
};

struct InitialPosePerception
{
    int agentId;
    int x;
    int y;
    int fieldSize;
    bool hasArrow;
    int heading;
};

struct ActionResponsePerception
{
    int agentId;
    int x;
    int y;
    int heading;
    std::vector<int> responses;
};

struct InitialPoseRequestData
{
    int agentId;
    int startX;
    int startY;
};

struct ActionRequestData
{
    int agentId;
    int action;
};

struct AgentPosition
{
    int agentId;
    int startX;
    int startY;
    int heading;
};

struct MultiInitialPoseRequestData
{
    std::vector<wumpus::model::communication::AgentPosition> agentPositions;
};

struct LoadWorldRequestData
{
    std::string worldPath;
};
}
}
}
