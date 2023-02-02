cmake_minimum_required(VERSION 3.0.2)
project(sim_msgs)

find_package(catkin REQUIRED
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
  FILES
  Action.msg
  VHA.msg
)
add_service_files(
  FILES
  MoveArm.srv
  AttachObj.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_msgs
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
