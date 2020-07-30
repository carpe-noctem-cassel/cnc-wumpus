#include "DomainBehaviour.h"

using std::string;

namespace alica
{
DomainBehaviour::DomainBehaviour(std::string name)
        : BasicBehaviour(name)
{
    wm = wumpus::WumpusWorldModel::getInstance();
    sc = essentials::SystemConfig::getInstance();
    ros::NodeHandle n;

    wumpusActionRequestTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.ActionRequest", NULL);
    wumpusPoseRequestTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.SpawnAgentRequest", NULL);
    agentPerceptionTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.AgentPerception", NULL);
    multiPoseRequestTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.SpawnMultiAgentRequest", NULL);
    loadWorldRequestTopic = (*sc)["WumpusWorldModel"]->get<string>("Send.LoadWorldRequest", NULL);

    wumpusActionPublisher = n.advertise<wumpus_simulator::ActionRequest>(wumpusActionRequestTopic, 10);
    wumpusPosePublisher = n.advertise<wumpus_simulator::InitialPoseRequest>(wumpusPoseRequestTopic, 10);
    agentPerceptionPublisher = n.advertise<wumpus_msgs::AgentPerception>(agentPerceptionTopic, 10);
    multiAgentPoseRequestPublisher = n.advertise<wumpus_simulator::MultiInitialPoseRequest>(multiPoseRequestTopic, 10);
    loadWorldRequestPublisher = n.advertise<wumpus_simulator::LoadWorldRequest>(loadWorldRequestTopic, 10);
}

void DomainBehaviour::send(wumpus_simulator::InitialPoseRequest& msg)
{
    wumpusPosePublisher.publish(msg);
}

void DomainBehaviour::send(wumpus_simulator::ActionRequest& msg)
{
    wumpusActionPublisher.publish(msg);
}
void DomainBehaviour::send(wumpus_msgs::AgentPerception& msg)
{
    agentPerceptionPublisher.publish(msg);
}

void DomainBehaviour::send(wumpus_simulator::MultiInitialPoseRequest& msg)
{
    multiAgentPoseRequestPublisher.publish(msg);
}

void DomainBehaviour::send(wumpus_simulator::LoadWorldRequest& msg)
{
    loadWorldRequestPublisher.publish(msg);
}

wumpus_msgs::AgentPerception DomainBehaviour::createAgentPerception(int ownId) const
{
    auto localAgent = wm->playground->getAgentById(ownId);
    auto currentPos = localAgent->currentPosition;
    wumpus_msgs::AgentPerception perception;
    wumpus_msgs::Coordinates coords;
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
    std::stringstream obj;
    obj << localAgent->objective;
    perception.objective = obj.str();
    perception.shot = localAgent->shot;
    perception.receivedScream = localAgent->receivedScreamForOwnShot;
    perception.receivedSilence = localAgent->receivedSilenceForOwnShot;
    //    auto shotAtFields = wm->playground->getFieldsShotAtByAgentIds();
    //    if (shotAtFields->find(essentials::SystemConfig::getOwnRobotID()) != shotAtFields->end()) {
    //        for (const auto& field : shotAtFields->at(essentials::SystemConfig::getOwnRobotID())) {
    //            wumpus_msgs::Coordinates coordinates;
    //            coordinates.x = field->x;
    //            coordinates.y = field->y;
    //            perception.shootingTargets.push_back(coordinates);
    //        }
    //    }
    for (const auto& shotAt : localAgent->shotAtFields) {
        wumpus_msgs::Coordinates coordinates;
        coordinates.x = shotAt->x;
        coordinates.y = shotAt->y;
        perception.shootingTargets.push_back(coordinates);
    }
    for (const auto& blockingWumpus : localAgent->fieldsWithBlockingWumpi) {
        wumpus_msgs::Coordinates coordinates;
        coordinates.x = blockingWumpus->x;
        coordinates.y = blockingWumpus->y;
        perception.blockingWumpi.emplace_back(coordinates);
    }
    for (const auto& blockingTrap : localAgent->fieldsWithBlockingTraps) {
        wumpus_msgs::Coordinates coordinates;
        coordinates.x = blockingTrap->x;
        coordinates.y = blockingTrap->y;
        perception.blockingTraps.emplace_back(coordinates);
    }
    for (const auto& possibleNext : localAgent->possibleNextFields) {
        wumpus_msgs::Coordinates coordinates;
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
