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
import importlib
from std_srvs.srv import SetBool, SetBoolResponse
from gazebo_msgs.srv import SetLinkState, SetLinkStateRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')


class Zone:
    def __init__(self, id, x1, y1, x2, y2, pose, valid_actions) -> None:
        self.id = id
        # Right-Upper corner (x1,y1)
        self.x1 = x1
        self.y1 = y1
        # Left-Lower corner (x2,y2)
        self.x2 = x2
        self.y2 = y2

        self.world_pose = pose

        self.valid_actions = valid_actions

        self.current_action_id = -10

        self.is_pass = False

    def setPixelCoords(self, x1,y1,x2,y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __repr__(self) -> str:
        return f"{self.id}-({self.x1},{self.y1})-({self.x2},{self.y2})"

g_vha = VHA()
g_vha_received = False
g_step_over = False
g_timeout_max = 0
g_best_human_action = 0


# Set predefined pose
q_all = Quaternion(*quaternion_from_euler(0, 0.541, 0))
q_0 = Quaternion(*quaternion_from_euler(0, 0, 0))
g_far_zone_pose = Pose(Point(0,0,-2), q_0)
g_init_pose_zones = {
    0: Pose(Point(0.99, 0.29, 1),  q_all),
    1: Pose(Point(1.06, -0.33, 1), q_all),
    2: Pose(Point(1.36, -0.58, 1), q_all),
    3: Pose(Point(1.36, -0.43, 1), q_all),
    4: Pose(Point(1.36, -0.17, 1), q_all),
    5: Pose(Point(1.45, 0.44, 1),  q_all),
}
# Create zones
g_zones = {
    0:Zone(0, -1, -1, -1, -1, g_init_pose_zones[0], ["place"]),
    1:Zone(1, -1, -1, -1, -1, g_init_pose_zones[1], ["pick('y', 'C')", "drop('y',)"]),
    2:Zone(2, -1, -1, -1, -1, g_init_pose_zones[2], ["pick('r', 'H')", "drop('r',)"]),
    3:Zone(3, -1, -1, -1, -1, g_init_pose_zones[3], ["pick('b', 'H')", "drop('b',)"]),
    4:Zone(4, -1, -1, -1, -1, g_init_pose_zones[4], ["pick('p', 'H')", "drop('p',)"]),
    5:Zone(5, -1, -1, -1, -1, g_init_pose_zones[5], ["PASS"]),
}
g_zones[5].is_pass = True
# Get and set zones pixels 
f = open('/home/afavier/exec_simulator_ws/src/gazebo_plugin/zones_coords.txt', 'r')
for l in f:
    l = l[:-1]
    if l=="":
        continue
    l = l.split(',')
    id,x1,y1,x2,y2 = [int(i) for i in l]
    g_zones[id].setPixelCoords(x1,y1,x2,y2)

def isInZone(pose: Point, zone: Zone):
    return pose.x>=zone.x1 and pose.x<=zone.x2 and pose.y>=zone.y1 and pose.y<=zone.y2

def hide_all_zones():
    srv = SetModelStateRequest()
    for z in g_zones.values():
        srv.model_state.model_name = f"z{z.id}"
        srv.model_state.pose = g_far_zone_pose
        srv.model_state.reference_frame = "world" 
        g_set_model_state_client(srv)

def show_all_zones():
    srv = SetModelStateRequest()
    for z in g_zones.values():
        srv.model_state.model_name = f"z{z.id}"
        srv.model_state.pose = z.world_pose
        srv.model_state.reference_frame = "world" 
        g_set_model_state_client(srv)
    

#########
## ROS ##
#########
def incoming_vha_cb(msg: VHA):
    global g_vha, g_vha_received
    print("vha received")
    print(msg)
    g_vha = msg
    g_vha_received = True

    # update zones
    for z in g_zones.values():
        if z.is_pass and g_vha.type==VHA.NS: # PASS
            show = True
            z.current_action_id = -1
        else:
            show = False
            for i,ha in enumerate(g_vha.valid_human_actions):
                if ha in z.valid_actions or ha[:len("place")] in z.valid_actions:
                    show = True
                    z.current_action_id = i+1
                    break

        srv = SetModelStateRequest()
        srv.model_state.model_name = f"z{z.id}"
        srv.model_state.reference_frame = "world"
        if show:
            srv.model_state.pose = z.world_pose
        else:
            srv.model_state.pose = g_far_zone_pose
            z.current_action_id = -10
        g_set_model_state_client(srv)
        
def mouse_pressed_cb(msg: Point):
    zone_clicked = None
    for z in g_zones.values():
        if z.current_action_id!=-10 and isInZone(msg, z):
            zone_clicked = z
            break
    print(f"zone clicked: {zone_clicked}")

    if zone_clicked!=None:
        g_start_human_action_prox(zone_clicked.current_action_id)
        hide_all_zones()
        print("human decision sent")


##########
## MAIN ##
##########

def main():
    global g_vha, g_vha_received, g_step_over, g_timeout_max, g_best_human_action, g_human_choice_pub, g_set_model_state_client, g_start_human_action_prox

    rospy.init_node('mouse_human', log_level=rospy.INFO)

    human_vha_sub = rospy.Subscriber("hmi_vha", VHA, incoming_vha_cb, queue_size=1)
    click_sub = rospy.Subscriber("/mouse_pressed_pose", Point, mouse_pressed_cb, queue_size=1)

    started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
    timeout_max_service = rospy.Service("hmi_timeout_max", Int, lambda req: IntResponse())
    r_idle_service = rospy.Service("hmi_r_idle", SetBool, lambda req: SetBoolResponse())

    rospy.wait_for_service("/gazebo/set_model_state")
    g_set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    

    # Spawn zones
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    spawn_zones_individually = True
    if spawn_zones_individually==False:
        # zones
        f = open('/home/afavier/exec_simulator_ws/src/simulator/worlds/zones.sdf','r')
        sdff = f.read()
        f.close()
        spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_model_prox("zones", sdff, "", Pose(), "world")
        print(f"zones spawned")

    else:
        for i in range(6):
            f = open(f'/home/afavier/exec_simulator_ws/src/simulator/worlds/z{i}.sdf','r')
            sdff = f.read()
            f.close()
            spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
            spawn_model_prox(f"z{i}", sdff, "", g_far_zone_pose, "world")
            print(f"z{i} spawned")
        # show_all_zones()

    rospy.wait_for_service("start_human_action")
    g_start_human_action_prox = rospy.ServiceProxy("start_human_action", Int)

    rospy.spin()

if __name__ == "__main__":
    main()


