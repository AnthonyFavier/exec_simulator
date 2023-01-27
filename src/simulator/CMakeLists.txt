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
)

find_package(Eigen3)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
    DEPENDS
        gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

add_executable(attach_box src/attach_box.cpp)
target_link_libraries(attach_box ${catkin_LIBRARIES})


add_executable(attach_box_eigen src/attach_box_eigen.cpp)
target_link_libraries(attach_box_eigen ${catkin_LIBRARIES})

add_executable(pick_box src/pick_box.cpp)
target_link_libraries(pick_box ${catkin_LIBRARIES})


