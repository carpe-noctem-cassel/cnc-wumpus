#include "EvalBase.h"

#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>

#include <aspkb/TermManager.h>
#include <reasoner/asp/IncrementalExtensionQuery.h>

#include <ros/ros.h>

#include <chrono>
#include <iostream>
#include <thread>

namespace wumpus
{

EvalBase::EvalBase(std::string roleSetName, std::string masterPlanName, std::string roleSetDir, std::string worldName, int startX, int startY)
{
    ae = new alica::AlicaEngine(new essentials::AgentIDManager(new essentials::AgentIDFactory()), roleSetName, masterPlanName, false);
    bc = new alica::BehaviourCreator();
    cc = new alica::ConditionCreator();
    crc = new alica::ConstraintCreator();
    uc = new alica::UtilityFunctionCreator();

    ae->setAlicaClock(new alicaRosProxy::AlicaROSClock());
    ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));

    solver = new ::reasoner::asp::Solver({});
    auto solverWrapper = new alica::reasoner::ASPSolverWrapper(ae, {});
    solverWrapper->init(solver);

    ae->addSolver(solverWrapper);

    aspkb::TermManager::getInstance().initializeSolver(solver);

    wm = WumpusWorldModel::getInstance();

    wm->setEngine(ae);
    wm->init();

    ae->init(bc, cc, uc, crc);
}

void EvalBase::start()
{
    ae->start();
}

EvalBase::~EvalBase()
{
    ae->shutdown();
    wm->clear();
    aspkb::TermManager::getInstance().clear();
    //delete ae->getAlicaClock();
    ::reasoner::asp::IncrementalExtensionQuery::queries.clear();
    delete ae->getCommunicator();
    delete ae;
    delete bc;
    delete cc;
    delete crc;
    delete uc;
    delete solver;
}

} /* namespace wumpus */
