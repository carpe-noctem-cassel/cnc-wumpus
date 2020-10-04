#include "DomainBehaviour.h"
#include <wumpus/model/communication/AgentPerceptions.h>
using std::string;

namespace alica
{
DomainBehaviour::DomainBehaviour(std::string name)
        : BasicBehaviour(name)
{
    wm = wumpus::WumpusWorldModel::getInstance();
    sc = essentials::SystemConfig::getInstance();
}

void DomainBehaviour::send(wumpus::model::communication::InitialPoseRequestData data)
{
    this->wm->communication->sendInitialPoseRequest(data);
}

void DomainBehaviour::send(wumpus::model::communication::ActionRequestData data)
{
    this->wm->communication->sendActionRequest(data);
}
void DomainBehaviour::send(wumpus::model::communication::AgentPerceptions data)
{
    this->wm->communication->sendAgentPerceptions(data);
}

void DomainBehaviour::send(wumpus::model::communication::MultiInitialPoseRequestData data)
{
    this->wm->communication->sendMultiInitialPoseRequest(data);
}

void DomainBehaviour::send(wumpus::model::communication::LoadWorldRequestData data)
{
    this->wm->communication->sendLoadWorldRequest(data);
}

    wumpus::model::communication::AgentPerceptions DomainBehaviour::createAgentPerception(int ownId) const
{
    auto localAgent = wm->playground->getAgentById(ownId);
    auto currentPos = localAgent->currentPosition;
    wumpus::model::communication::AgentPerceptions perception;
    wumpus::model::communication::Coordinates coords;
    coords.x = currentPos->x;
    coords.y = currentPos->y;
    perception.position = coords;
    coords.x = localAgent->initialPosition->x;
    coords.y = localAgent->initialPosition->y;
    perception.initialPosition = coords;

    if (wm->localAgentIsSpawnRequestHandler()) {
        perception.encoding = wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding();
    } else {
        perception.encoding = wm->currentEncoding;
    }
    perception.stinky = currentPos->stinky;
    perception.glitter = currentPos->shiny;
    perception.drafty = currentPos->drafty;
    perception.senderID = ownId;
    perception.exited = wm->localAgentExited;
    perception.died = wm->localAgentDied;
    perception.haveGold = localAgent->hasGold;
    perception.exhausted = localAgent->exhausted;
    perception.objective = localAgent->objective;
    perception.shot = localAgent->shot;
    perception.receivedScream = localAgent->receivedScreamForOwnShot;
    perception.receivedSilence = localAgent->receivedSilenceForOwnShot;
    //    auto shotAtFields = wm->playground->getFieldsShotAtByAgentIds();
    //    if (shotAtFields->find(essentials::SystemConfig::getOwnRobotID()) != shotAtFields->end()) {
    //        for (const auto& field : shotAtFields->at(essentials::SystemConfig::getOwnRobotID())) {
    //            wumpus::model::communication::Coordinates coordinates;
    //            coordinates.x = field->x;
    //            coordinates.y = field->y;
    //            perception.shootingTargets.push_back(coordinates);
    //        }
    //    }
    for (const auto& shotAt : localAgent->shotAtFields) {
        wumpus::model::communication::Coordinates coordinates;
        coordinates.x = shotAt->x;
        coordinates.y = shotAt->y;
        perception.shootingTargets.push_back(coordinates);
    }
    for (const auto& blockingWumpus : localAgent->fieldsWithBlockingWumpi) {
        wumpus::model::communication::Coordinates coordinates;
        coordinates.x = blockingWumpus->x;
        coordinates.y = blockingWumpus->y;
        perception.blockingWumpi.emplace_back(coordinates);
    }
    for (const auto& blockingTrap : localAgent->fieldsWithBlockingTraps) {
        wumpus::model::communication::Coordinates coordinates;
        coordinates.x = blockingTrap->x;
        coordinates.y = blockingTrap->y;
        perception.blockingTraps.emplace_back(coordinates);
    }
    for (const auto& possibleNext : localAgent->possibleNextFields) {
        wumpus::model::communication::Coordinates coordinates;
        coordinates.x = possibleNext->x;
        coordinates.y = possibleNext->y;
        perception.possibleNextFields.emplace_back(coordinates);
    }
    std::cout << "Made own agent perception to send!" << std::endl;
    return perception;
}

DomainBehaviour::~DomainBehaviour()
{
}
} /* namespace alica */
