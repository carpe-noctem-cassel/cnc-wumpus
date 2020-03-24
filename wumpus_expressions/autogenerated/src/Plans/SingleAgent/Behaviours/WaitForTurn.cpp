using namespace std;
#include "Plans/SingleAgent/Behaviours/WaitForTurn.h"

/*PROTECTED REGION ID(inccpp1534835364093) ENABLED START*/ // Add additional includes here
#include <wumpus/model/Agent.h>
#include <wumpus/model/Field.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1534835364093) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
WaitForTurn::WaitForTurn()
        : DomainBehaviour("WaitForTurn")
{
    /*PROTECTED REGION ID(con1534835364093) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
WaitForTurn::~WaitForTurn()
{
    /*PROTECTED REGION ID(dcon1534835364093) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void WaitForTurn::run(void* msg)
{
    /*PROTECTED REGION ID(run1534835364093) ENABLED START*/ // Add additional options here
    // TODO experimental...
    if (this->wm->localAgentDied || this->wm->localAgentExited) {
        wumpus_msgs::AgentPerception perception;
        wumpus_msgs::Coordinates pos;
        pos.x = -1;
        pos.y = -1;
        perception.initialPosition = pos;
        perception.position = pos;
        perception.senderID = sc->getOwnRobotID();
        perception.died = this->wm->localAgentDied;
        if (this->wm->localAgentDied) {
            auto localAgent = this->wm->playground->getAgentById(essentials::SystemConfig::getOwnRobotID());
            if (localAgent) {
                if (localAgent->diedOn) {
                    wumpus_msgs::Coordinates diedOn;
                    diedOn.x = localAgent->diedOn->x;
                    diedOn.y = localAgent->diedOn->y;
                    perception.diedOn = diedOn;
                } else {
                    std::cout << "Reset: Can't access field the agent died on!" << std::endl;
                    throw std::exception();
                }
            }
        }
        perception.exited = this->wm->localAgentExited;
        if (this->wm->localAgentIsSpawnRequestHandler()) {
            perception.encoding = this->wm->experiment->getCurrentRun()->getCurrentStartPositionsEncoding();
        }
        perception.exhausted = this->wm->playground->getAgentById(sc->getOwnRobotID())->exhausted;
        perception.shot = this->wm->playground->getAgentById(sc->getOwnRobotID())->shot;
        auto shotAtFields = this->wm->getShotAtFields();
        if (shotAtFields->find(essentials::SystemConfig::getOwnRobotID()) != shotAtFields->end()) {
            for (const auto& field : shotAtFields->at(essentials::SystemConfig::getOwnRobotID())) {
                wumpus_msgs::Coordinates coordinates;
                coordinates.x = std::stoi(field.first); // FIXME types
                coordinates.y = std::stoi(field.second);
                perception.shootingTargets.push_back(coordinates);
            }
        }
        send(perception);
    }
    /*PROTECTED REGION END*/
}
void WaitForTurn::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1534835364093) ENABLED START*/ // Add additional options here
    //        std::cout << "WaitForTurn: waiting for turn" << std::endl;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1534835364093) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
