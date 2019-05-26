using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/SingleAgent/Behaviours/InitASPModel.h"

#include  "Plans/SingleAgent/Behaviours/EvaluateGoal.h"

#include  "Plans/SingleAgent/Behaviours/GenerateActions.h"

#include  "Plans/SingleAgent/Behaviours/EvaluateActions.h"

#include  "Plans/SingleAgent/Behaviours/SpawnAgent.h"

#include  "Plans/SingleAgent/Behaviours/WaitForTurn.h"

#include  "Plans/SingleAgent/Behaviours/DetermineObjective.h"

#include  "Plans/SingleAgent/Behaviours/PerformNextAction.h"

#include  "Plans/SingleAgent/Behaviours/PerformAction.h"

#include  "Plans/SingleAgent/Behaviours/DetermineGoal.h"

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

            case 1551695098593:

                return make_shared<EvaluateGoal>();
                break;

            case 1551695113275:

                return make_shared<GenerateActions>();
                break;

            case 1551695131246:

                return make_shared<EvaluateActions>();
                break;

            case 1534835358495:

                return make_shared<SpawnAgent>();
                break;

            case 1534835374750:

                return make_shared<WaitForTurn>();
                break;

            case 1554202577790:

                return make_shared<DetermineObjective>();
                break;

            case 1551695161214:

                return make_shared<PerformNextAction>();
                break;

            case 1534836780649:

                return make_shared<PerformAction>();
                break;

            case 1551695076041:

                return make_shared<DetermineGoal>();
                break;

            default:
                cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
                throw new exception();
                break;
        }
    }
}
