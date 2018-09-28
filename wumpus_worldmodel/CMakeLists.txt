cmake_minimum_required(VERSION 2.8.3)
project(wumpus_worldmodel)
    
## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
 
## Enable exception handling for segfaults using SigFault.h
set(CMAKE_CXX_FLAGS "-rdynamic -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS system_config event_handling alica_engine wumpus_simulator world_model asp_solver_wrapper asp_solver asp_commons)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wumpus_worldmodel
  CATKIN_DEPENDS system_config event_handling world_model wumpus_simulator asp_solver_wrapper asp_solver asp_commons
)
 
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(wumpus_worldmodel

  src/wumpus/wm/Coordinates.cpp
  src/wumpus/wm/Field.cpp
  src/wumpus/WumpusWorldModel.cpp
  src/wumpus/wm/WumpusSimData.cpp
  src/wumpus/wm/WumpusAgent.cpp
  src/wumpus/wm/Communication.cpp
  src/wumpus/wm/ASPKnowledgeBase.cpp
  src/wumpus/wm/Playground.cpp
  src/wumpus/wm/KnowledgeManager.cpp

)

#add_dependencies(wumpus_worldmodel geometry_msgs_generate_messages_cpp robot_control_generate_messages_cpp) 

target_link_libraries(wumpus_worldmodel
  ${catkin_LIBRARIES} 
)
