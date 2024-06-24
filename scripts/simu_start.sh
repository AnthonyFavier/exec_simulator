#!/bin/bash

export GAZEBO_PLUGIN_PATH=`rospack find exec_automaton`/../gazebo_plugin/build
roslaunch tiago_gazebo tiago_gazebo.launch