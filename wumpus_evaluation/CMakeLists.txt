cmake_minimum_required(VERSION 2.8.3)
project(wumpus_evaluation)

find_package(catkin REQUIRED COMPONENTS
        wumpus_simulator
        system_config
        rqt_gui
        rqt_gui_cpp
        roscpp
        qt_gui
        system_util
        fsystem
        event_handling
        alica_engine
        alica_ros_proxy
        wumpus_expressions
        wumpus_worldmodel
        robot_control

        asp_solver_wrapper
        asp_solver) #wumpus_base

find_package(Qt5Core REQUIRED)
get_target_property(Qt5Core_location Qt5::Core LOCATION)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
    set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
    set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
    add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

# Dummy target for IDE's
#FILE (GLOB_RECURSE all_headers_for_ides
#        ${CMAKE_SOURCE_DIR}/**/*.hpp
#        ${CMAKE_SOURCE_DIR}/**/*.h)
#ADD_CUSTOM_TARGET(ide_includes SOURCES ${all_headers_for_ides})
#
#
#set(wwf_generator_INCLUDE_DIRECTORIES
#        include
#        ../wumpus_simulator/include
#        ${Qt5Core_INCLUDE_DIRS}
#        #"${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
#        )

#if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

#file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
#endif()

#QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT5_WRAP_CPP(wumpus_MOCS ${wumpuswidget_HDRS})
# ensure generated header files are being created in the devel space
#set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
#set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
#QT5_WRAP_UI(wumpus_UIS_H ${wumpuswidget_UIS})
#set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

catkin_package(
        INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS wumpus_simulator
)

include_directories(include ${catkin_INCLUDE_DIRS} wumpus_simulator_INCLUDE_DIRECTORIES sim/include)

add_executable(${PROJECT_NAME} main.cpp sim/src/WumpusSimulatorHeadless.cpp sim/src/EvalBase.cpp)

#add_dependencies(${PROJECT_NAME} wumpus_simulator )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Qt5Core_location} clingo) #wumpus_simulator)
