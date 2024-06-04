--------------------------------------------------------------------
Building:
#1 Building the ros workspace (Repeat in case of error)
$ catkin build

#2 Building the gazebo_plugin
$ cd src/gazebo_plugin
$ mkdir build
$ cd build
$ cmake ..
$ make

#3 Install custom Progress package
$ cd src/progress
$ python setup.py install --user 

--------------------------------------------------------------------
Launching the simulation is a several step process, each done in a different shell.
This implies that the planning and characterization processes have already been done.

# Sourcing (for each shell)
$ source devel/setup.bash

# Shell 1 - Start the simulator, moveit processes and prompt window
$ ./script/full_start.sh
A few warnings and the following error are expected: "[Err] [msgs.cc:2873] Unrecognized geometry type".
You shall keep the prompt window on the foreground. To do so, right click on the window while pressing the Super/Window key, and select "Always on Top".
After, start the prompt node in the prompt window:
$ ./scripts/prompt_node.sh 
Finally, remove the tool bars of the simulator with "Ctrl+H".

# Shell 2 - Start the controllers (move_arm, move_hand, simulation actions)
$ roslaunch simulator control.launch

# Shell 3 - Start the execution automaton
$ roslaunch exec_automaton exec_automaton.launch

# Shell 4 - Start the mouse human HMI
$ rosrun mouse_human mouse_human.py
OR start the Mock Human
$ rosrun mock_human mock_human.py

# [Optional] Shell 5 - Start timeline record
$ rosrun exec_automaton timeline_log.py record|load