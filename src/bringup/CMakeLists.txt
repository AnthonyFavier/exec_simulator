cmake_minimum_required(VERSION 3.0.2)
project(bringup)

find_package(catkin REQUIRED COMPONENTS
    rqt_joint_trajectory_controller
    moveit_ros_planning_interface 
    rviz_visual_tools
    moveit_visual_tools
)

catkin_package(
    # LIBRARIES
    # INCLUDE_DIRS
    # CATKIN_DEPENDS
    #     moveit_visual_tools
    # DEPENDS
)

include_directories(
#   include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_group_interface_tutorial src/test.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})