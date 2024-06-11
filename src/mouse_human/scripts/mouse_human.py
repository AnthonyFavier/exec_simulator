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

DOMAIN_NAME = "epistemic"

# path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
# sys.path.insert(0, path)

## LOGGER ##
# logging.config.fileConfig(path + 'log.conf')


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

        # for pass zone
        self.is_pass = False
        self.ready_activate_auto_pass = False
        self.ns_idle = False

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
g_prompt_button_pose = Pose(Point(1.3, 0.71, 1.37), q_all)
g_can_click_indicator_pose = Pose(Point(1.87, 0.0, 1.44), q_all)

# Create zones
g_zones = {}
def create_zone(id, x, y, w, h, list_actions):
    global g_zones
    g_zones[id] = Zone(id, None, list_actions)
    g_zones[id].setPixelCoords(x, y, x+w, y+h)

if DOMAIN_NAME=="epistemic":
    create_zone(0,      0,0,500,1080,       ["change_focus_towards"])
    create_zone(1,      0,0,1920,300,       ["move_to_table"])
    create_zone(2,      901,548,122,141,    ["pick"])
    create_zone(3,      634,374,312,378,    ["place_1('w1', 'box_1')"])
    create_zone(4,      973,374,312,378,    ["place_1('w1', 'box_2')"])
    create_zone(5,      0,0,0,0,            ["communicate_if_cube_can_be_put"])
    create_zone(6,      1371,820,228,217,   ["PASS"])
else:
    raise Exception("Domain_name unknown...")

# Detect PASS zone
for z in g_zones.values():
    if z.valid_actions==["PASS"]:
        break
g_z_pass = z
g_z_pass.is_pass = True

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
g_prompt_button_zone.setPixelCoords(1664, 299, 1899, 417)
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

def show_tuto_zones(req = None):
    srv = SetModelStateRequest()
    srv.model_state.model_name="t1"
    srv.model_state.pose = Pose(Point(1.8, 0.17, 1.5), q_all)
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    srv.model_state.model_name="t2"
    srv.model_state.pose = Pose(Point(2.0, -0.057, 1.655), q_all)
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    srv.model_state.model_name="t3"
    srv.model_state.pose = Pose(Point(2.0, -0.055, 1.57), q_all)
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    srv.model_state.model_name="t4"
    srv.model_state.pose = Pose(Point(2.15, -0.108, 1.595), q_all)
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()
def hide_tuto_zones(req = None):
    srv = SetModelStateRequest()
    srv.model_state.model_name="t1"
    srv.model_state.pose = g_far_zone_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    srv.model_state.model_name="t2"
    srv.model_state.pose = g_far_zone_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    srv.model_state.model_name="t3"
    srv.model_state.pose = g_far_zone_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    srv.model_state.model_name="t4"
    srv.model_state.pose = g_far_zone_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()

def show_auto_pass_indicator(req = None):
    srv = SetModelStateRequest()
    srv.model_state.model_name="auto_pass_indicator"
    srv.model_state.pose = Pose(Point(2.1, 0.22, 1.495), q_all)
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()
def hide_auto_pass_indicator(req = None):
    srv = SetModelStateRequest()
    srv.model_state.model_name="auto_pass_indicator"
    srv.model_state.pose = g_far_zone_pose
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()

def reset_auto_pass_cb(msg):
    global AUTO_PASS
    AUTO_PASS = False

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

    # Update pass zone
    # print(g_vha)
    if g_vha.type==VHA.NS:
        g_z_pass.current_action_id = -1
        g_z_pass.ready_activate_auto_pass = False
        g_z_pass.ns_idle = False
    elif g_vha.type==VHA.NS_IDLE: 
        g_z_pass.current_action_id = -10
        g_z_pass.ready_activate_auto_pass = True
        g_z_pass.ns_idle = True
    elif g_vha.type==VHA.CONCURRENT: # CONCURRENT (already PASS or TO)
        g_z_pass.current_action_id = -10
        g_z_pass.ready_activate_auto_pass = True
        g_z_pass.ns_idle = False

    # Update action zones
    for z in g_zones.values():
        if z.is_pass:
            continue
        z.current_action_id = -10
        for i,ha in enumerate(g_vha.valid_human_actions):
            for z_a in z.valid_actions:
                if z_a == ha[:len(z_a)]: # ha starts with z_a
                    z.current_action_id = i+1
                    break
            if z.current_action_id != -10:
                break

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
    global AUTO_PASS

    rospy.init_node('mouse_human', log_level=rospy.INFO)

    human_vha_sub = rospy.Subscriber("hmi_vha", VHA, incoming_vha_cb, queue_size=1)
    click_sub = rospy.Subscriber("/mouse_pressed_pose", Point, mouse_pressed_cb, queue_size=1)
    TO_reached_sub = rospy.Subscriber("/hmi_timeout_reached", EmptyM, TO_reached_cb, queue_size=1)
    reset_auto_pass_sub = rospy.Subscriber("/reset_auto_pass", EmptyM, reset_auto_pass_cb, queue_size=1)

    timeout_max_service = rospy.Service("hmi_timeout_max", Int, lambda req: IntResponse())
    r_idle_service = rospy.Service("hmi_r_idle", SetBool, lambda req: SetBoolResponse())
    show_zones_service = rospy.Service("show_zones", EmptyS, show_all_zones)
    hide_zones_service = rospy.Service("hide_zones", EmptyS, hide_all_zones)
    show_prompt_button_service = rospy.Service("show_prompt_button", EmptyS, show_prompt_button)
    hide_prompt_button_service = rospy.Service("hide_prompt_button", EmptyS, hide_prompt_button)
    show_tuto_zones_service = rospy.Service("show_tuto_zones", EmptyS, show_tuto_zones)
    hide_tuto_zones_service = rospy.Service("hide_tuto_zones", EmptyS, hide_tuto_zones)
    show_auto_pass_indicator_service = rospy.Service("show_auto_pass_indicator", EmptyS, show_auto_pass_indicator)
    hide_auto_pass_indicator_service = rospy.Service("hide_auto_pass_indicator", EmptyS, hide_auto_pass_indicator)

    auto_pass_pub = rospy.Publisher("/auto_pass_state", Bool, queue_size=1)


    reset_last_click_service = rospy.Service("reset_last_click", EmptyS, reset_last_click)

    g_prompt_button_pressed_pub = rospy.Publisher('/prompt_button_pressed', EmptyM, queue_size=1)


    rospy.wait_for_service("/gazebo/set_model_state")
    g_set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)


    # SPAWNING #
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    # spawn prompt button
    f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/prompt_button.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model_prox(f"prompt_button", sdff, "", g_far_zone_pose, "world")
    print(f"prompt_button spawned")

    # spawn tuto zones
    f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/t1.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model_prox(f"t1", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/t2.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model_prox(f"t2", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/t3.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model_prox(f"t3", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/t4.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model_prox(f"t4", sdff, "", g_far_zone_pose, "world")

    # spawn auto_click indicator
    f = open(f'/home/afavier/new_exec_sim_ws/src/simulator/worlds/auto_pass_indicator.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model_prox(f"auto_pass_indicator", sdff, "", g_far_zone_pose, "world")

    ##################

    started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
    rospy.loginfo("HMI ready")

    rospy.wait_for_service("start_human_action")
    g_start_human_action_prox = rospy.ServiceProxy("start_human_action", Int)

    AUTO_PASS = False

    while not rospy.is_shutdown():

        # PROMPT BUTTON
        if g_prompt_button_shown:
            if g_prompt_button_zone.clickIn(g_last_click):
                rospy.loginfo("prompt button pressed !")
                g_prompt_button_pressed_pub.publish(EmptyM())
                hide_prompt_button()
                reset_last_click()

        else:

            if AUTO_PASS==False:
                # Check if AUTO_PASS being activated
                if g_z_pass.ready_activate_auto_pass and g_z_pass.clickIn(g_last_click):
                    rospy.logwarn("Activating AUTO_PASS")
                    AUTO_PASS = True
                    auto_pass_pub.publish(True)
                    reset_last_click()
                    show_auto_pass_indicator()

                # ACTION ZONES
                for z in g_zones.values():
                    if z.isActive() and z.clickIn(g_last_click):
                        decision_sent = True
                        rospy.loginfo(f"zone clicked: {z}")
                        g_start_human_action_prox(z.current_action_id)
                        hide_all_zones()
                        rospy.loginfo("human decision sent")
                        reset_last_click()
                        break
            
            if AUTO_PASS==True:

                # Check if AUTO_PASS is being disabled
                if g_z_pass.clickIn(g_last_click):
                    rospy.logwarn("Disactivating AUTO_PASS")
                    AUTO_PASS = False
                    auto_pass_pub.publish(False)
                    reset_last_click()
                    hide_auto_pass_indicator()

                # Make auto pass if possible
                elif g_z_pass.isActive():
                    decision_sent = True
                    g_start_human_action_prox(g_z_pass.current_action_id)
                    hide_all_zones()
                    reset_last_click()

                # Human should act, even in AUTO_PASS
                elif g_z_pass.ns_idle:
                    for z in g_zones.values():
                        if z.isActive() and z.clickIn(g_last_click):
                            decision_sent = True
                            rospy.loginfo(f"zone clicked: {z}")
                            g_start_human_action_prox(z.current_action_id)
                            hide_all_zones()
                            rospy.loginfo("human decision sent")
                            reset_last_click()
                            break

        # TODO: Sound click was ineffective

        time.sleep(0.01)
            
if __name__ == "__main__":
    main()


