#include "wumpus/wm/Communication.h"

#include "wumpus/WumpusWorldModel.h"

#include <essentials/AgentID.h>
#include <essentials/BroadcastID.h>
#include <wumpus/model/communication/AgentPerceptions.h>
#include <wumpus/wm/util/ROSCommunicationUtil.h>
#include <wumpus_simulator/MultiInitialPoseResponse.h>
namespace wumpus
{
namespace wm
{

Communication::Communication(wumpus::WumpusWorldModel* wm)
        : wm(wm)
{
    auto sc = wm->getSystemConfig();
    // SET ROS STUFF
    std::string topic;

    topic = (*sc)["WumpusWorldModel"]->get<std::string>("Data.InitialPoseResponse.Topic", NULL);
    initialPoseResponseSub = n.subscribe(topic, 10, &Communication::onInitialPoseResponse, (Communication*) this);

    topic = (*sc)["WumpusWorldModel"]->get<std::string>("Data.ActionResponse.Topic", NULL);
    actionResponseSub = n.subscribe(topic, 10, &Communication::onActionResponse, (Communication*) this);

    topic = (*sc)["WumpusWorldModel"]->get<std::string>("Data.AgentPerception.Topic", NULL);
    agentPerceptionSub = n.subscribe(topic, 10, &Communication::onAgentPerception, (Communication*) this);

    ros::NodeHandle n;

    wumpusActionRequestTopic = (*sc)["WumpusWorldModel"]->get<std::string>("Send.ActionRequest", NULL);
    wumpusPoseRequestTopic = (*sc)["WumpusWorldModel"]->get<std::string>("Send.SpawnAgentRequest", NULL);
    agentPerceptionTopic = (*sc)["WumpusWorldModel"]->get<std::string>("Send.AgentPerception", NULL);
    multiPoseRequestTopic = (*sc)["WumpusWorldModel"]->get<std::string>("Send.SpawnMultiAgentRequest", NULL);
    loadWorldRequestTopic = (*sc)["WumpusWorldModel"]->get<std::string>("Send.LoadWorldRequest", NULL);

    wumpusActionPublisher = n.advertise<wumpus_simulator::ActionRequest>(wumpusActionRequestTopic, 10);
    wumpusPosePublisher = n.advertise<wumpus_simulator::InitialPoseRequest>(wumpusPoseRequestTopic, 10);
    agentPerceptionPublisher = n.advertise<wumpus_msgs::AgentPerception>(agentPerceptionTopic, 10);
    multiAgentPoseRequestPublisher = n.advertise<wumpus_simulator::MultiInitialPoseRequest>(multiPoseRequestTopic, 10);
    loadWorldRequestPublisher = n.advertise<wumpus_simulator::LoadWorldRequest>(loadWorldRequestTopic, 10);

    this->multiAgentSpawnResponseSub =
            n.subscribe("wumpus_simulator/SpawnMultiAgentResponse", 10, &Communication::onMultiInitialPoseResponse, (Communication*) this);

    auto wumpusActionRequestTopic = (*sc)["WumpusWorldModel"]->get<std::string>("Send.ActionRequest", NULL);

    this->timeoutPublisher = n.advertise<wumpus_simulator::ActionRequest>(wumpusActionRequestTopic, 10);

    spinner = new ros::AsyncSpinner(4);
    spinner->start();
}

Communication::~Communication()
{
    spinner->stop();
    delete spinner;
}

void Communication::onInitialPoseResponse(wumpus_simulator::InitialPoseResponsePtr initialPoseResponse)
{
    this->wm->wumpusSimData.processInitialPoseResponse(wumpus::wm::util::ROSCommunicationUtil::toInitialPosePerception(initialPoseResponse));
}

void Communication::onActionResponse(wumpus_simulator::ActionResponsePtr actionResponse)
{
    this->wm->wumpusSimData.processActionResponse(wumpus::wm::util::ROSCommunicationUtil::toActionResponsePerception(actionResponse));
}

void Communication::onAgentPerception(wumpus_msgs::AgentPerceptionPtr agentPerception)
{
    this->wm->wumpusSimData.processAgentPerception(wumpus::wm::util::ROSCommunicationUtil::toAgentPerception(agentPerception));
}

void Communication::onMultiInitialPoseResponse(wumpus_simulator::MultiInitialPoseResponsePtr response)
{
    if (!response->success) {
        std::cout << "Pose request failed!" << std::endl;
        auto id = this->wm->getEngine()->getTeamManager()->getLocalAgentID();
        if (this->wm->getEngine()->getRoleAssignment()->getRole(id)->getName() == this->wm->spawnRequestHandlerRoleName) {
            this->wm->experiment->getCurrentRun()->getCurrentResult()->completionStatus = eval::CompletionStatus::REJECTED;
        }

        this->wm->experiment->getCurrentRun()->setSpawnRequestStatus(
                this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding(), eval::SpawnRequestStatus::FAILED);
    } else {
        this->wm->experiment->getCurrentRun()->setSpawnRequestStatus(
                this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding(), eval::SpawnRequestStatus::ACCEPTED);
    }
}

void Communication::sendAgentPerception(wumpus_msgs::AgentPerception& msg)
{
    agentPerceptionPublisher.publish(msg);
}

/**
 * TODO remove after debugging! timeout is not supposed to happen
 */
void Communication::sendTimeoutMessage()
{
    std::cout << "IN SEND TIMEOUT MESSAGE" << std::endl;
    wumpus_simulator::ActionRequest msg;
    msg.agentId = essentials::SystemConfig::getOwnRobotID();
    msg.action = WumpusEnums::actions::timeoutRequest;
    this->timeoutPublisher.publish(msg);
}

void Communication::sendInitialPoseRequest(wumpus::model::communication::InitialPoseRequestData data)
{
    wumpusPosePublisher.publish(wumpus::wm::util::ROSCommunicationUtil::toInitialPoseRequestMsg(data));
}

void Communication::sendActionRequest(wumpus::model::communication::ActionRequestData data)
{
    wumpusActionPublisher.publish(wumpus::wm::util::ROSCommunicationUtil::toActionRequestMsg(data));
}
void Communication::sendAgentPerceptions(wumpus::model::communication::AgentPerceptions data)
{
    agentPerceptionPublisher.publish(wumpus::wm::util::ROSCommunicationUtil::toAgentPerceptionMsg(data));
}

void Communication::sendMultiInitialPoseRequest(wumpus::model::communication::MultiInitialPoseRequestData data)
{
    multiAgentPoseRequestPublisher.publish(wumpus::wm::util::ROSCommunicationUtil::toMultiInitialPoseRequestMsg(data));
}

void Communication::sendLoadWorldRequest(wumpus::model::communication::LoadWorldRequestData data)
{
    loadWorldRequestPublisher.publish(wumpus::wm::util::ROSCommunicationUtil::toLoadWorldRequestMsg(data));
}
} /* namespace wm */
} /* namespace wumpus */
