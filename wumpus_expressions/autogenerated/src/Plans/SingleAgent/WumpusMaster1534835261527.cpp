#include "Plans/SingleAgent/WumpusMaster1534835261527.h"
using namespace alica;
/*PROTECTED REGION ID(eph1534835261527) ENABLED START*/ //Add additional using directives here
#include "wumpus/WumpusWorldModel.h"
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:WumpusMaster

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1534835261530

     */
    shared_ptr<UtilityFunction> UtilityFunction1534835261527::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1534835261527) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: Stop in Plan: WumpusMaster

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : spawnAgent success 
     *
     * Plans in State: 				
     *   - Plan - (Name): SpawnAgentDefault, (PlanID): 1534835358495 
     *
     * Tasks: 
     *   - DefaultTask (1414681164704) (Entrypoint: 1534835261530)
     *
     * States:
     *   - Stop (1534835261528)
     *   - Play (1534836316488)
     *   - AgentLeftCave (1534836405246)
     *   - AgentDead (1534836429671)
     *
     * Vars:
     */
    bool TransitionCondition1534836324356::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1534836323236) ENABLED START*/
    	auto initialPose = wm->wumpusSimData.getInitialPoseResponseBuffer()->getLastValidContent();
        if(initialPose) {
        	if(initialPose->agentId == sc->getOwnRobotID()) {
        		return true;
        	}
        }
        return false;
        /*PROTECTED REGION END*/

    }

    //State: Play in Plan: WumpusMaster

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : leave cave success 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - DefaultTask (1414681164704) (Entrypoint: 1534835261530)
     *
     * States:
     *   - Stop (1534835261528)
     *   - Play (1534836316488)
     *   - AgentLeftCave (1534836405246)
     *   - AgentDead (1534836429671)
     *
     * Vars:
     */
    bool TransitionCondition1534836415069::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1534836414368) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : agent dead 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - DefaultTask (1414681164704) (Entrypoint: 1534835261530)
     *
     * States:
     *   - Stop (1534835261528)
     *   - Play (1534836316488)
     *   - AgentLeftCave (1534836405246)
     *   - AgentDead (1534836429671)
     *
     * Vars:
     */
    bool TransitionCondition1534836447768::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1534836446000) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

//State: AgentLeftCave in Plan: WumpusMaster

//State: AgentDead in Plan: WumpusMaster

}