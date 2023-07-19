Building:
#1 Building the ros workspace
$ catkin_make

#2 Building the gazebo_plugin
$ cd src/gazebo_plugin/build
$ cmake ..
$ make

#3 Install Progress package
$ cd src/progress
$ python setup.py install --user 

Launching the simulation is a 4 step process.
This implies that the planning and characterization processes has been already ran.

#0 Sourcing
$ source devel/setup.bash
$ source export_gaz_plugin_path.sh

#1 Start the simulator and moveit processes
$ roslaunch bringup bringup.launch

#2 Start the controllers (move_arm, move_hand, simulation actions)
$ roslaunch simulator control.launch

#3 Start the execution automaton
$ roslaunch exec_automaton exec_automaton.launch

#4 Start the HMI
$ rosrun simple_hmi hmi.py

#4_bis Start the Mock Human
$ rosrun mock_human mock_human.py