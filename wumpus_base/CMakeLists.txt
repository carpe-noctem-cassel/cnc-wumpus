cmake_minimum_required(VERSION 2.8.3)
project(wumpus_base)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

## Enable exception handling for segfaults
set(CMAKE_CXX_FLAGS "-rdynamic -g -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  system_config 
  system_util 
  fsystem 
  event_handling
  alica_engine 
  alica_ros_proxy 
  wumpus_expressions 
  wumpus_worldmodel
  robot_control 

  asp_solver_wrapper
  asp_solver
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wumpus_base
  CATKIN_DEPENDS alica_engine alica_ros_proxy wumpus_expressions wumpus_worldmodel 
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wumpus_base src/Base.cpp)

target_link_libraries(wumpus_base ${catkin_LIBRARIES} clingo)
