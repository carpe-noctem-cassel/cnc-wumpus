#pragma once

#include <engine/AlicaEngine.h>

#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <UtilityFunctionCreator.h>

#include <wumpus/WumpusWorldModel.h>

#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <reasoner/asp/Solver.h>


#include <iostream>

namespace wumpus
{

    class EvalBase
    {
    public:
        EvalBase(std::string roleSetName, std::string masterPlanName, std::string roleSetDir, std::string worldName, int startX, int startY);
        virtual ~EvalBase();

        void start();

        alica::AlicaEngine* ae;
        alica::BehaviourCreator* bc;
        alica::ConditionCreator* cc;
        alica::ConstraintCreator* crc;
        alica::UtilityFunctionCreator* uc;

        reasoner::asp::Solver* solver;


        WumpusWorldModel* wm;

    };

} /* namespace wumpus */
