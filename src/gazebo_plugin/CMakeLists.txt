cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(MyGazeboPlugin)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(CameraLockPlugin SHARED ${headers_MOC}
  CameraLockPlugin.cc)
target_link_libraries(CameraLockPlugin ${GAZEBO_LIBRARIES})

add_library(MousePosePublisherPlugin SHARED ${headers_MOC}
  MousePosePublisherPlugin.cc)
target_link_libraries(MousePosePublisherPlugin ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES})

add_library(GUITextPlugin SHARED ${headers_MOC}
  GUITextPlugin.cc)
target_link_libraries(GUITextPlugin ${GAZEBO_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${PROTOBUF_LIBRARIES})  