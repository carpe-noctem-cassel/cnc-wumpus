# Wumpus AI 

----
## System Configuration
* Ubuntu 16.04
* ROS Kinetic
* alica [master], last commit 26abe4a4fb13ffdcc3cea075c3fcef13f8fe4649
* essentials [ttb_dev], last commit 31d3f37f4dde6da70a088f4a3eb7bd85fc6839b0
* alica-supplementary [master], last commit ad09b58492a38eb4eba4fc19044a51a155b2f97c
* aspsuite  [wumpus_dev], most recent
* wumpus_simulator [wumpus\_dev], most recent
* wumpus_ai [master], most recent

----
## Running a simulation

* Start roscore
* Start the Wumpus Simulator 
<!-- -->

    rosrun wumpus_simulator wumpus_simulator

* Create a new world or load an existing one (examples are in 'worlds' folder

* Set the number of agents to be spawned in etc/WumpusWorldModel.conf (default 1)

* Start the base of each agent
<!-- -->

    ROBOT=agent rosrun wumpus_base wumpus_base -m WumpusMaster

(Agent must be defined in etc/Globals.conf. Currently available: agent, tweety and pingu)


----
## Creating worlds with wwf_generator

Navigate to the target directory of the generated worlds.
Start roscore.

### With config values:

* Open and edit etc/WWFGenerator.conf 
* Run wwf\_generator without Command Line arguments

<!-- -->
    rosrun wwf_generator wwf_generator

### or: Configuration from command line arguments: 

    rosrun wwf_generator wwf_generator -n [num_worlds] -s [fixed playground size] -w [num_wumpi] -t [num_traps] -a [agent starts with arrow, 0 or 1]

Generated worlds will be unique.

----

## Running wumpus_evaluation

This uses a non-UI version of the Wumpus Simulator to run multiple simulations one after another (until agent dies or exits). On finish, it prints how many worlds have been completed successfully.

* Navigate to a directory containing ONLY .wwf files
* Start roscore
* Start the evaluation
<!-- -->
    ROBOT=agent rosrun wumpus_evaluation wumpus_evaluation
