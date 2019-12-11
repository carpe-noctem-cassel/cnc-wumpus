#include "Base.h"

#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>

#include <asp_solver_wrapper/ASPSolverWrapper.h>
#include <reasoner/asp/Solver.h>

#include <aspkb/TermManager.h>

#include <SigFault.h>

#include <ros/ros.h>

#include <chrono>
#include <iostream>
#include <thread>

namespace wumpus
{

Base::Base(std::string roleSetName, std::string masterPlanName, std::string roleSetDir)
{
    ae = new alica::AlicaEngine(new essentials::AgentIDManager(new essentials::AgentIDFactory()), roleSetName, masterPlanName, false);
    bc = new alica::BehaviourCreator();
    cc = new alica::ConditionCreator();
    crc = new alica::ConstraintCreator();
    uc = new alica::UtilityFunctionCreator();

    ae->setAlicaClock(new alicaRosProxy::AlicaROSClock());
    ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));

    auto solver = new ::reasoner::asp::Solver({});
    auto solverWrapper = new alica::reasoner::ASPSolverWrapper(ae, {});
    solverWrapper->init(solver);

    ae->addSolver(solverWrapper);

    aspkb::TermManager::getInstance().initializeSolver(solver);

    wm = WumpusWorldModel::getInstance();

    wm->setEngine(ae);
    wm->init();

    ae->init(bc, cc, uc, crc);

}

void Base::start()
{
    ae->start();
}

Base::~Base()
{
    ae->shutdown();
    delete ae->getAlicaClock();
    delete ae->getCommunicator();
    delete ae;
    delete bc;
    delete cc;
    delete crc;
    delete uc;
}

} /* namespace wumpus */

void printUsage()
{
    std::cout << "Usage: ./msl_base -m \"Masterplan\" [-rd \"RoleSetDirectory\"] [-rset \"RoleSet\"]" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        printUsage();
        return 0;
    }

    std::cout << "Initialising ROS" << std::endl;

    ros::init(argc, argv, essentials::SystemConfig::getInstance()->getHostname() + "_Base");

    // This makes segfaults to exceptions
    segfaultdebug::init_segfault_exceptions();

    std::cout << "Parsing command line parameters:" << std::endl;

    std::string masterplan = "";
    std::string rolesetdir = ".";
    std::string roleset = "";

    for (int i = 1; i < argc; i++) {
        if (string(argv[i]) == "-m" || string(argv[i]) == "-masterplan") {
            masterplan = argv[i + 1];
            i++;
        }

        if (string(argv[i]) == "-rd" || string(argv[i]) == "-rolesetdir") {
            rolesetdir = argv[i + 1];
            i++;
        }
        if (string(argv[i]) == "-r" || string(argv[i]) == "-roleset") {
            roleset = argv[i + 1];
            i++;
        }
    }
    if (masterplan.size() == 0 || rolesetdir.size() == 0) {
        printUsage();
        return 0;
    }
    std::cout << "\tMasterplan is:       \"" << masterplan << "\"" << std::endl;
    std::cout << "\tRolset Directory is: \"" << rolesetdir << "\"" << std::endl;
    std::cout << "\tRolset is:           \"" << (roleset.empty() ? "Default" : roleset) << "\"" << std::endl;

    std::cout << "\nConstructing Base ..." << std::endl;
    wumpus::Base* base = new wumpus::Base(roleset, masterplan, rolesetdir);

    std::cout << "\nStarting Base ..." << std::endl;
    base->start();

    while (ros::ok()) {
        std::chrono::milliseconds dura(500);
        std::this_thread::sleep_for(dura);
    }

    return 0;
}
