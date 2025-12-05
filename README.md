# HRI PR2 cubes simulation

## 1.Docker environment setting
First of all, install Docker on your computer and follows the following instructions.
1. move your terminal to ```docker_exec_simulator/docker```
2. replace in Dockerfile ```UNAME``` and ```REPO_PATH```, respectively, with your pc local username and absolute path to the cloned repository (no `\` at the end).
3. replace ALL files the tag `<absolute_path_to_replace>` by the same value as ```REPO_PATH```.
4. run ```./build_docker.sh```
5. run ```xhost +local:root```
6. run ```./run_docker.sh``` (comment ```--gpus all \``` if you don't have a gpu)
Now, in your terminal you have a virtual environment suitable to build and run the original repository. IMPORTANTE: the commands below has to be all executed in the terminal of the virtual environment!



## 2.Building ROS workspace (move to the path of docker_exec_simulator)
```$ source /opt/ros/noetic/setup.bash```

```$ catkin build``` (usually it is necessary to run it twice the first trial to overcome errors)

## 3.Building the gazebo_plugin
```$ cd src/gazebo_plugin```

```$ mkdir build```

```$ cd build```

```$ cmake ..```

```$ make```

## 4.Install custom Progress package
```$ cd src/progress```

```$ python setup.py install --user ```

## txt[Optional] Make Gazebo exit faster

o make Gazebo die sooner, edit the file at `/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py`:

Near line 57, change the timeouts to be:

```
_TIMEOUT_SIGINT  = 0.5 #seconds
_TIMEOUT_SIGTERM = 0.5 #seconds
```

This will cause ROS to send a `kill` signal much sooner.

https://gist.github.com/plusk01/7b50443138a8d4b0ff742fa86b75c1e5

## Note

Sound is disabled for now, and all command `.play()` are commented in `exec_automaton.py`.
Currently not able to execute in docker.

--------------------------------------------------------------------

# Launch

1. run ```xhost +local:root```
2. run ```./run_docker.sh```

Launching the simulation is a several step process, each done in a different shell.
This implies that the planning and characterization processes have already been done (access to `<policy_name>.p` file).

## Setup different shell

Several processes must be started. When using docker, find the name or ID of the docker by running `docker ps`.
Then, for each new terminal connect to the original docker instance, move to working folder, and source:
```
$ docker exec -it <docker_name/ID> bash 
$ cd <working_folder>
$ source devel/setup.bash
```

## Shell 1 - Start the simulator, moveit processes and prompt window
```$ ./scripts/full_start.sh```

A few warnings and the following error are expected: "[Err] [msgs.cc:2873] Unrecognized geometry type".
If the scene if dark in Gazebo, go in the side panel first, go in scene, and check and uncheck the shadow box.
Then make sure it is fullscreen and remove the tool bars with "Ctrl+H"

## Shell 2 - Start prompt window

You shall keep the prompt window on the foreground. To do so, right click on the window while pressing the Super/Window key, and select "Always on Top".
After, start the prompt node in the prompt window:

```$ ./scripts/prompt_node.sh ```

## Shell 3 - Start the controllers (move_arm, move_hand, simulation actions)
```$ roslaunch simulator control.launch```

## Shell 4 - Start the execution automaton
```$ roslaunch exec_automaton exec_automaton.launch```

## Shell 5 - Start the mouse human HMI
```$ rosrun mouse_human mouse_human.py```

OR start the Mock Human

```$ rosrun mock_human mock_human.py```

## txt[Optional] Shell 6 - Start timeline record
```$ rosrun exec_automaton timeline_log.py record|load```



Additional information on the use arugments and environmental variables can be checked in this discussion: https://stackoverflow.com/questions/40902445/using-variable-interpolation-in-string-in-docker/40902661
