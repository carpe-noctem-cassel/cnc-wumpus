/**
 * T License (MIT)
 *
 * Copyright (c) 2018 Distributed Systems Group, University of Kassel, Germany
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <wumpus_simulator/ActionRequest.h>
#include <wumpus_simulator/ActionResponse.h>
#include <wumpus_simulator/InitialPoseRequest.h>
#include <wumpus_simulator/InitialPoseResponse.h>

#include <ros/macros.h>
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include <iostream>

namespace wumpus_simulator
{

class Model;
class GroundTile;
class Agent;
class Wumpus;
namespace headless
{

/**
 * Handles interactions with agent and wumpus.
 */
class WumpusSimulatorHeadless
{

public:
    WumpusSimulatorHeadless();
    ~WumpusSimulatorHeadless();


    /**
     * Initializes world
     * @param arrow bool agent has arrow?
     * @param size string fieldsize nxn
     * @param traps string number of traps
     * @param wumpus string number of wumpus
     */
    void createWorld(bool arrow, int wumpus, int traps, int size);

    /**
     * Loads a wwf file
     */
    void loadWorld(std::string world);

    Model* getModel();

    // ROS Stuff
    ros::NodeHandle n;
    ros::AsyncSpinner* spinner;

    ros::Subscriber spawnAgentSub;
    ros::Subscriber actionSub;

    ros::Publisher spawnAgentPub;
    ros::Publisher actionPub;

private:
    Model* model;
    bool ready;
    int turnIndex;
    std::vector<int> turns;

    /**
     * Colors playground according to model
     */
    void updatePlayground();

    /**
     * Handles incoming spawn request
     */
    void onSpawnAgent(InitialPoseRequestPtr msg);

    /**
     * Handles incoming action request and calls corresponding handle method
     */
    void onAction(ActionRequestPtr msg);

    /**
     * Places agent randomly on a free field
     * @param agentId int positive id for agent
     */
    void placeAgent(int agentId, bool hasArrow);

    /**
     * Enables steering of already placed wumpus
     * @param wumpusId int negative id for wumpus
     */
    void possessWumpus(int wumpusId);

    /**
     * Delegates mesg to corresponding method
     */
    void handleAction(ActionRequestPtr msg);

    /**
     * Call method according to given message
     */
    void handleWumpusAction(ActionRequestPtr msg);

    /**
     * Turns the agent right by 90 degrees
     */
    void handleTurnRight(ActionRequestPtr msg);

    /**
     * Turns the agent left by 90 degrees
     */
    void handleTurnLeft(ActionRequestPtr msg);

    /**
     * Shoots an arrow in the direction of the agent's current heading
     */
    void handleShoot(ActionRequestPtr msg);

    /**
     * Handles the request to pick up gold
     */
    void handlePickUpGold(ActionRequestPtr msg);

    /**
     * Handles the request to leave the playground.
     * Agent can only leave the playground if they
     * have collected the gold and are standing on
     * their starting position
     */
    void handleExit(ActionRequestPtr msg);

    /**
     * Moves the agent reminding the outer walls
     */
    void handleMove(ActionRequestPtr msg);

    /*
     * Informs the next agent or wumpus
     */
    void handleNextTurn();

    /**
     * Informs agent about breeze, stench and glitter
     */
    void handlePerception(ActionResponse& msg, std::shared_ptr<GroundTile> tile);

    /**
     * Shoots an arrow to the left killing all wumpus on its way
     */
    void handleShootLeft(ActionResponse& msg, std::shared_ptr<Agent> agent);

    /**
     * Shoots an arrow to the right killing all wumpus on its way
     */
    void handleShootRight(ActionResponse& msg, std::shared_ptr<Agent> agent);

    /**
     * Shoots an arrow upwards killing all wumpus on its way
     */
    void handleShootUp(ActionResponse& msg, std::shared_ptr<Agent> agent);

    /**
     * Shoots an arrow downwards killing all wumpus on its way
     */
    void handleShootDown(ActionResponse& msg, std::shared_ptr<Agent> agent);

    /**
     * Kills wumpus and removes it from turns
     */
    void killWumpus(std::shared_ptr<Wumpus> wumpus);

    /**
     * Kills agent and removes it from turns
     */
    void killAgent(std::shared_ptr<Agent> agent);

    /**
     * Advances turn index
     */
    void getNext();
};
}
}
