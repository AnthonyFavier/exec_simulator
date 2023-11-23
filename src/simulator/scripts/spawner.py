#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Dict, List, Tuple
from copy import deepcopy
import random
import dill
import sys
from enum import Enum
import logging as lg
import logging.config
import rospy
from sim_msgs.msg import Action, VHA
from std_msgs.msg import Int32, Bool, Float64
from std_msgs.msg import Empty as EmptyM
from sim_msgs.srv import Int, IntResponse, IntRequest
from std_srvs.srv import Empty as EmptyS
from std_srvs.srv import EmptyResponse
from progress.bar import IncrementalBar
from sim_msgs.msg import EventLog
from std_srvs.srv import SetBool, SetBoolResponse
from gazebo_msgs.srv import SetLinkState, SetLinkStateRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time



q_0 = Quaternion(*quaternion_from_euler(0, 0, 0))

##########
## MAIN ##
##########

            
if __name__ == "__main__":
    rospy.init_node('mouse_human', log_level=rospy.INFO)

    # SPAWNING #
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    # spawn prompt button
    f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/scripts/test_cube.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model_prox(f"test_cube", sdff, "", Pose(Point(0.86,-0.5,0.75), q_0), "world")

