cmake_minimum_required(VERSION 3.0.2)
project(simulator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    gazebo_ros
    moveit_ros_planning_interface
    rviz_visual_tools
    moveit_visual_tools
    sim_msgs
)

find_package(Eigen3)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
    DEPENDS
        gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

install(PROGRAMS
  scripts/ompl_constrained_planning_tutorial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(EXEC_LIST 
attach_obj
attach_obj_tf
pick_box
init_static_scene
new_move_multiple
move_arm
sim_controller
)

foreach(NODE ${EXEC_LIST})
    add_executable(${NODE} src/${NODE}.cpp)
    target_link_libraries(${NODE} ${catkin_LIBRARIES})
endforeach()


