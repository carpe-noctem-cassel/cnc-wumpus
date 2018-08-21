#pragma once

#include <iostream>

#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <UtilityFunctionCreator.h>
#include <ConstraintCreator.h>
#include <engine/AlicaEngine.h>
#include <wumpus/WumpusWorldModel.h>
using namespace std;


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
		alica::UtilityFunctionCreator* uc;
		alica::ConstraintCreator* crc;
		WumpusWorldModel* wm;

	protected:
	};

} /* namespace wumpus */
