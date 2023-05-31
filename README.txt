Launching the simulation is a 4 step process.
This implies that the planning and characterization processes has been already ran.

#1 Start the simulator and moveit processes
roslaunch bringup bringup.launch

#2 Start the controllers (move_arm, move_hand, simulation actions)
roslaunch simulator control.launch

#3 Start the execution automaton
rosrun exec_automaton exec_automaton.py

#4 Start the HMI
rosrun simple_hmi hmi.py

#4_bis Start the Mock Human
rosrun mock_human mock_human.py