cmake_minimum_required(VERSION 2.8.3)
project(wumpus_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Coordinates.msg
   AgentPerception.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wumpus_msgs
  CATKIN_DEPENDS message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

