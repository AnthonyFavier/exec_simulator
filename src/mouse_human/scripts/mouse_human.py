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

DOMAIN_NAME = "stack_empiler_2"

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

    def clickIn(self, click):
        if click==None:
            return False
        return click.pose.x>=self.x1 and click.pose.x<=self.x2 and click.pose.y>=self.y1 and click.pose.y<=self.y2

    def isActive(self):
        return self.current_action_id!=-10

    def __repr__(self) -> str:
        return f"{self.id}-({self.x1},{self.y1})-({self.x2},{self.y2})-{self.valid_actions}"
    
class Click:
    def __init__(self, pose, time):
        self.pose = pose #type: Point
        self.time = time #type: float

g_vha = VHA()
g_vha_received = False
g_step_over = False
g_timeout_max = 0
g_best_human_action = 0


# Set predefined pose
q_all = Quaternion(*quaternion_from_euler(0, 1.0, 0))
q_0 = Quaternion(*quaternion_from_euler(0, 0, 0))
g_far_zone_pose = Pose(Point(0,0,-2), q_0)
g_prompt_button_pose = Pose(Point(1.2, 0.75, 1.22), q_all)
g_can_click_indicator_pose = Pose(Point(1.87, 0.0, 1.44), q_all)

# Create zones
g_zones = {}
def create_zone(id, x, y, w, h, list_actions):
    global g_zones
    g_zones[id] = Zone(id, None, list_actions)
    g_zones[id].setPixelCoords(x, y, x+w, y+h)

if DOMAIN_NAME=="stack_empiler_2":
    create_zone(0,      1063,352,348,374,   ["place"])
    create_zone(1,      356,483,666,453,    ["drop"])
    create_zone(2,      912,585,98,120,     ["pick('w1',)"])
    create_zone(3,      794,585,96,120,     ["pick('o2',)"])
    create_zone(4,      674,585,101,120,    ["pick('y2',)"])
    create_zone(5,      441,762,131,137,    ["pick('b2',)"])
    create_zone(6,      582,759,296,138,    ["pick('p1',)"])
    create_zone(7,      1371,820,228,217,   ["PASS"])
else:
    raise Exception("Domain_name unknown...")

# Detect PASS zone
for z in g_zones.values():
    if z.valid_actions==["PASS"]:
        z.is_pass = True
        break

def hide_all_zones(req = None):
    # srv = SetModelStateRequest()
    # first disable zone fast
    for z in g_zones.values():
        z.current_action_id = -10
    # Then move them away
    # for z in g_zones.values():
    #     srv.model_state.model_name = f"z{z.id}"
    #     srv.model_state.pose = g_far_zone_pose
    #     srv.model_state.reference_frame = "world" 
    #     g_set_model_state_client(srv)
    return EmptyResponse()

def show_all_zones(req = None):
    # print("showing zones")
    # srv = SetModelStateRequest()
    # for z in g_zones.values():
    #     srv.model_state.model_name = f"z{z.id}"
    #     srv.model_state.pose = z.world_pose
    #     srv.model_state.reference_frame = "world" 
    #     g_set_model_state_client(srv)
    return EmptyResponse()

g_prompt_button_shown = False
g_prompt_button_zone = Zone(-20, g_prompt_button_pose, None)
g_prompt_button_zone.setPixelCoords(1634, 401, 1843, 506)
def show_prompt_button(req = None):
    global g_prompt_button_shown
    g_prompt_button_shown = True
    srv = SetModelStateRequest()
    srv.model_state.model_name = "prompt_button"
    srv.model_state.pose = g_prompt_button_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)

    reset_last_click()
    
    return EmptyResponse()
def hide_prompt_button(req = None):
    global g_prompt_button_shown
    g_prompt_button_shown = False
    srv = SetModelStateRequest()
    srv.model_state.model_name = "prompt_button"
    srv.model_state.pose = g_far_zone_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()

g_can_click_indicator_shown = False
def hide_can_click_indicator(req = None):
    global g_can_click_indicator_shown
    g_can_click_indicator_shown = False
    srv = SetModelStateRequest()
    srv.model_state.model_name = "can_click_indicator"
    srv.model_state.pose = g_far_zone_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()
def show_can_click_indicator(req = None):
    global g_can_click_shown
    g_can_click_shown = True
    srv = SetModelStateRequest()
    srv.model_state.model_name = "can_click_indicator"
    srv.model_state.pose = g_can_click_indicator_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()


#########
## ROS ##
#########
def incoming_vha_cb(msg: VHA):
    global g_vha, g_vha_received, decision_sent
    decision_sent = False
    print("\n")
    rospy.loginfo("vha received")
    print(msg)
    g_vha = msg
    g_vha_received = True

    # update zones
    for z in g_zones.values():
        if z.is_pass and g_vha.type==VHA.NS: # PASS
            active = True
            z.current_action_id = -1
        else:
            active = False
            for i,ha in enumerate(g_vha.valid_human_actions):
                for z_a in z.valid_actions:
                    if z_a == ha[:len(z_a)]: # ha starts with z_a
                        active = True
                        z.current_action_id = i+1
                        break
                if active:
                    break

        # srv = SetModelStateRequest()
        # srv.model_state.model_name = f"z{z.id}"
        # srv.model_state.reference_frame = "world"
        if active:
            # srv.model_state.pose = z.world_pose
            pass
        else:
            # srv.model_state.pose = g_far_zone_pose
            z.current_action_id = -10
        # g_set_model_state_client(srv)

    if g_vha.valid_human_actions==[]:
        # hide_can_click_indicator()
        pass
    else:
        # show_can_click_indicator()
        if msg.timeout!=0.0:
            rospy.loginfo("Start timeout....")

            s_t = time.time()
            while not rospy.is_shutdown() and not decision_sent and time.time()-s_t<msg.timeout:
                time.sleep(0.01)
            
            if not decision_sent:
                rospy.loginfo("timeout reached")
                rospy.loginfo("start hiding zones")
                hide_all_zones()
                # hide_can_click_indicator()
                rospy.loginfo("done hiding zones")
            else:
                rospy.loginfo("timeout canceled")

decision_sent = False
g_last_click = None # type: None | Click
def mouse_pressed_cb(msg: Point):
    global g_last_click
    g_last_click = Click(msg, time.time())
    rospy.loginfo(f"click: {msg.x},{msg.y}")
def reset_last_click(req = None):
    global g_last_click
    g_last_click = None
    rospy.loginfo("click reset")
    return EmptyResponse()

def TO_reached_cb(msg: EmptyM):
    hide_all_zones()
    # hide_can_click_indicator()

##########
## MAIN ##
##########

def main():
    global g_vha, g_vha_received, g_step_over, g_timeout_max, g_best_human_action, g_human_choice_pub, g_set_model_state_client, g_start_human_action_prox
    global g_prompt_button_pressed_pub
    global decision_sent

    rospy.init_node('mouse_human', log_level=rospy.INFO)

    human_vha_sub = rospy.Subscriber("hmi_vha", VHA, incoming_vha_cb, queue_size=1)
    click_sub = rospy.Subscriber("/mouse_pressed_pose", Point, mouse_pressed_cb, queue_size=1)
    TO_reached_sub = rospy.Subscriber("/hmi_timeout_reached", EmptyM, TO_reached_cb, queue_size=1)

    timeout_max_service = rospy.Service("hmi_timeout_max", Int, lambda req: IntResponse())
    r_idle_service = rospy.Service("hmi_r_idle", SetBool, lambda req: SetBoolResponse())
    show_zones_service = rospy.Service("show_zones", EmptyS, show_all_zones)
    hide_zones_service = rospy.Service("hide_zones", EmptyS, hide_all_zones)
    show_prompt_button_service = rospy.Service("show_prompt_button", EmptyS, show_prompt_button)
    hide_prompt_button_service = rospy.Service("hide_prompt_button", EmptyS, hide_prompt_button)
    # show_can_click_indicator_service = rospy.Service("show_can_click_indicator", EmptyS, show_can_click_indicator)
    # hide_can_click_indicator_service = rospy.Service("hide_can_click_indicator", EmptyS, hide_can_click_indicator)

    reset_last_click_service = rospy.Service("reset_last_click", EmptyS, reset_last_click)

    g_prompt_button_pressed_pub = rospy.Publisher('/prompt_button_pressed', EmptyM, queue_size=1)


    rospy.wait_for_service("/gazebo/set_model_state")
    g_set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)


    # SPAWNING #
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    # Spawn zones
    # nb_zones = 13
    # for i in range(nb_zones+1):
    #     f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/z{i}.sdf','r')
    #     sdff = f.read()
    #     f.close()
    #     spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    #     spawn_model_prox(f"z{i}", sdff, "", g_far_zone_pose, "world")
    #     print(f"z{i} spawned")
    
    # spawn prompt button
    f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/prompt_button.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model_prox(f"prompt_button", sdff, "", g_far_zone_pose, "world")
    print(f"prompt_button spawned")

    # spawn can_click_indicator
    # f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/can_click_indicator_new.sdf','r')
    # sdff = f.read()
    # f.close()
    # spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    # spawn_model_prox(f"can_click_indicator", sdff, "", g_far_zone_pose, "world")
    # print(f"can_click_indicator spawned")


    started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
    rospy.loginfo("HMI ready")

    rospy.wait_for_service("start_human_action")
    g_start_human_action_prox = rospy.ServiceProxy("start_human_action", Int)

    while not rospy.is_shutdown():

        if g_prompt_button_shown:
            if g_prompt_button_zone.clickIn(g_last_click):
                rospy.loginfo("prompt button pressed !")
                g_prompt_button_pressed_pub.publish(EmptyM())
                hide_prompt_button()
                reset_last_click()

        else:
            for z in g_zones.values():
                if z.isActive() and z.clickIn(g_last_click):
                    decision_sent = True
                    rospy.loginfo(f"zone clicked: {z}")
                    g_start_human_action_prox(z.current_action_id)
                    hide_all_zones()
                    # hide_can_click_indicator()
                    rospy.loginfo("human decision sent")
                    reset_last_click()
                    break

        # TODO: Sound click was ineffective

        time.sleep(0.01)
            
if __name__ == "__main__":
    main()


