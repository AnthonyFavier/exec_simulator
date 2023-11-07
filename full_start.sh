#!/bin/bash
export GAZEBO_PLUGIN_PATH=`rospack find exec_automaton`/../gazebo_plugin/build
# window prompt
sh `rospack find exec_automaton`/../../prompt_window.sh
# roslaunch bringup full_bringup.launch
roslaunch tiago_gazebo tiago_gazebo.launch