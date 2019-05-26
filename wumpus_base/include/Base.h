#pragma once


#include <engine/AlicaEngine.h>

#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <UtilityFunctionCreator.h>

#include <wumpus/WumpusWorldModel.h>

#include <iostream>

namespace wumpus
{

	class Base
	{
	public:
		Base(std::string roleSetName, std::string masterPlanName, std::string roleSetDir);
		virtual ~Base();

		void start();

		alica::AlicaEngine* ae;
		alica::BehaviourCreator* bc;
		alica::ConditionCreator* cc;
		alica::ConstraintCreator* crc;
		alica::UtilityFunctionCreator* uc;

		WumpusWorldModel* wm;

	protected:
	};

} /* namespace wumpus */
