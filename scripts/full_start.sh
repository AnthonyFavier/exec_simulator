#!/bin/bash

# window prompt
sh `rospack find exec_automaton`/../../scripts/prompt_window.sh

# Simulator
export GAZEBO_PLUGIN_PATH=`rospack find exec_automaton`/../gazebo_plugin/build
roslaunch tiago_gazebo tiago_gazebo.launch