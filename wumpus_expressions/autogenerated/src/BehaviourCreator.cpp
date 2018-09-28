using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/SingleAgent/Behaviours/InitASPModel.h"

#include  "Plans/SingleAgent/Behaviours/SpawnAgent.h"

#include  "Plans/SingleAgent/Behaviours/WaitForTurn.h"

#include  "Plans/SingleAgent/Behaviours/PerformAction.h"

namespace alica
{

    BehaviourCreator::BehaviourCreator()
    {
    }

    BehaviourCreator::~BehaviourCreator()
    {
    }

    shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId)
    {
        switch (behaviourConfId)
        {

            case 1536061762657:

                return make_shared<InitASPModel>();
                break;

            case 1534835358495:

                return make_shared<SpawnAgent>();
                break;

            case 1534835374750:

                return make_shared<WaitForTurn>();
                break;

            case 1534836780649:

                return make_shared<PerformAction>();
                break;

            default:
                cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
                throw new exception();
                break;
        }
    }
}
