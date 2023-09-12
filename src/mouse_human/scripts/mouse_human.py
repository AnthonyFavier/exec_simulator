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

DOMAIN_NAME = "stack_empiler_1"

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')


class Zone:
    def __init__(self, id, pose, valid_actions) -> None:
        self.id = id
        # Right-Upper corner (x1,y1)
        self.x1 = -1
        self.y1 = -1
        # Left-Lower corner (x2,y2)
        self.x2 = -1
        self.y2 = -1

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
q_all = Quaternion(*quaternion_from_euler(0, 1.0, 0))
q_0 = Quaternion(*quaternion_from_euler(0, 0, 0))
g_far_zone_pose = Pose(Point(0,0,-2), q_0)

# Create zones
g_zones = {}
def create_zone(id, x, y, z, list_actions):
    global g_zones
    # list_actions: [act1, act2]
    g_zones[id] = Zone(id, Pose(Point(x, y, z), q_all), list_actions)

if DOMAIN_NAME=="stack_empiler":
    create_zone(0, 1.12, 0.30,  1.03, ["place"])
    create_zone(1, 1.11, -0.09, 0.93, ["pick('w1',)"])
    create_zone(2, 1.10, -0.40, 0.94, ["drop"])
    create_zone(3, 1.41, -0.44, 0.93, ["pick('b2',)"])
    create_zone(5, 1.55, 0.46,  0.95, ["PASS"])

elif DOMAIN_NAME=="stack_empiler_1":
    create_zone(0, 2.5, 0.05, 1.99,     ["place"])
    create_zone(1, 2.2, -0.03, 1.75,    ["pick('w1',)"])
    create_zone(2, 2.2, -0.15, 1.75,    ["drop"])
    create_zone(3, 2.25, -0.225, 1.7,   ["pick('g2',)"])
    create_zone(4, 2.25, -0.172, 1.7,   ["pick('b2',)"])
    create_zone(6, 2.25, -0.088, 1.7,   ["pick('p1',)"])
    create_zone(5, 2.5, 0.11, 1.9,      ["PASS"])

elif DOMAIN_NAME=="classic":
    create_zone(0, 1.12, 0.30, 1.03,    ["place"])
    create_zone(1, 1.11, -0.33, 0.93,   ["pick('y', 'C')", "drop('y',)"])
    create_zone(2, 1.42, -0.59, 0.94,   ["pick('r', 'H')", "drop('r',)"])
    create_zone(3, 1.41, -0.44, 0.93,   ["pick('b', 'H')", "drop('b',)"])
    create_zone(4, 1.41, -0.17, 0.92,   ["pick('p', 'H')", "drop('p',)"])
    create_zone(5, 1.55, 0.46, 0.95,    ["PASS"])

else:
    raise Exception("Domain_name unknown...")

for z in g_zones.values():
    if z.valid_actions==["PASS"]:
        z.is_pass = True
        break
# Get and set zones pixels 
f = open('/home/afavier/new_exec_sim_ws/src/gazebo_plugin/zones_'+DOMAIN_NAME+'_coords.txt', 'r')
for l in f:
    l = l[:-1]
    if l=="":
        continue
    l = l.split(',')
    id,x1,y1,x2,y2 = [int(i) for i in l]
    g_zones[id].setPixelCoords(x1,y1,x2,y2)

print(g_zones)

def isInZone(pose: Point, zone: Zone):
    return pose.x>=zone.x1 and pose.x<=zone.x2 and pose.y>=zone.y1 and pose.y<=zone.y2

def hide_all_zones(req = None):
    srv = SetModelStateRequest()
    for z in g_zones.values():
        srv.model_state.model_name = f"z{z.id}"
        srv.model_state.pose = g_far_zone_pose
        srv.model_state.reference_frame = "world" 
        g_set_model_state_client(srv)
    return EmptyResponse()

def show_all_zones(req = None):
    print("showing zones")
    srv = SetModelStateRequest()
    for z in g_zones.values():
        srv.model_state.model_name = f"z{z.id}"
        srv.model_state.pose = z.world_pose
        srv.model_state.reference_frame = "world" 
        g_set_model_state_client(srv)
    return EmptyResponse()

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
                for z_a in z.valid_actions:
                    if z_a == ha[:len(z_a)]:
                    # if z_a in ha:
                        show = True
                        z.current_action_id = i+1
                        break
                if show:
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

    show_zones_service = rospy.Service("show_zones", EmptyS, show_all_zones)
    hide_zones_service = rospy.Service("hide_zones", EmptyS, hide_all_zones)

    rospy.wait_for_service("/gazebo/set_model_state")
    g_set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)


    # Spawn zones
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    spawn_zones_individually = True
    if spawn_zones_individually==False:
        # zones
        f = open('/home/afavier/new_exec_sim_ws/src/simulator/worlds/zones.sdf','r')
        sdff = f.read()
        f.close()
        spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_model_prox("zones", sdff, "", Pose(), "world")
        print(f"zones spawned")

    else:
        nb_zones = 7
        for i in range(nb_zones+1):
            f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/z{i}.sdf','r')
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


