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
from std_msgs.msg import Int32, Bool, Float64, String
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
from sim_msgs.msg import BoxTypes
from sim_msgs.srv import SetBoxTypes, SetBoxTypesRequest, SetBoxTypesResponse
from sim_msgs.srv import GetBoxTypes, GetBoxTypesRequest, GetBoxTypesResponse
from sim_msgs.srv import SetQuestionButtons, SetQuestionButtonsRequest, SetQuestionButtonsResponse

DOMAIN_NAME = "epistemic"

# path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
# sys.path.insert(0, path)

## LOGGER ##
# logging.config.fileConfig(path + 'log.conf')

def quaternion_msgs_from_rpy(r,p,y):
    x,y,z,w = quaternion_from_euler(r,p,y)
    return Quaternion(x,y,z,w)


TURN_BUTTON_POSE_MAIN = Pose( Point(2.0, -0.35, 1.63), quaternion_msgs_from_rpy(3.14159, -0.6, 0))
MOVE_BUTTON_POSE_MAIN = Pose( Point(3.4, 0, 1.75), quaternion_msgs_from_rpy(1.57, 0.6, 0))

TURN_BUTTON_POSE_SIDE = Pose( Point(7.2, 0.35, 1.63), quaternion_msgs_from_rpy(0, 0.6, 0))
MOVE_BUTTON_POSE_SIDE = Pose( Point(5.8, 0, 1.75), quaternion_msgs_from_rpy(1.57, -0.6, 0))

class Zone:
    _ID = 0
    def __init__(self, pose, valid_actions) -> None:
        self.id = Zone._ID
        Zone._ID+=1
    
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
g_zones = []
def create_zone(x, y, w, h, list_actions):
    global g_zones
    new_zone = Zone(None, list_actions)
    new_zone.setPixelCoords(x, y, x+w, y+h)
    g_zones.append(new_zone)

if DOMAIN_NAME=="epistemic":
    create_zone(0,0,500,1080,           ["change_focus_towards"])
    create_zone(0,0,1920,300,           ["move_to_table"])
    create_zone(1206,519,100,115,       ["pick('w1',)"])
    create_zone(549,572,120,130,        ["pick('g1',)"])
    create_zone(765,396,264,351,        ["place_1('w1', 'box_1')"])
    create_zone(1030,396,251,351,       ["place_1('w1', 'box_2')"])
    create_zone(1291,396,257,351,       ["place_1('w1', 'box_3')"])
    create_zone(1371,820,228,217,       ["PASS"])
    create_zone(1476, 296, 110, 109,    ["q1"])
    create_zone(1638,296,110,109,       ["q2"])
    create_zone(1800,296,108,108,       ["q3"])
    create_zone(1640,452,109,110,       ["q4"])
    create_zone(1804,452,110,109,       ["q5"])
else:
    raise Exception("Domain_name unknown...")


# Detect Special zones
for z in g_zones:
    if z.valid_actions==["PASS"]:
        g_z_pass = z
        g_z_pass.is_pass = True
    elif z.valid_actions==["q1"]:
        g_z_q1 = z
    elif z.valid_actions==["q2"]:
        g_z_q2 = z
    elif z.valid_actions==["q3"]:
        g_z_q3 = z
    elif z.valid_actions==["q4"]:
        g_z_q4 = z
    elif z.valid_actions==["q5"]:
        g_z_q5 = z

def disable_zones(req = None):
    # first disable zone fast
    for z in g_zones:
        z.current_action_id = -10

    hide_move_buttons()

    return EmptyResponse()

g_prompt_button_shown = False
g_prompt_button_zone = Zone(g_prompt_button_pose, None)
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

def show_turn_buttons(req = None):
    srv = SetModelStateRequest()
    srv.model_state.model_name="move_button_main"
    srv.model_state.pose = TURN_BUTTON_POSE_MAIN
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    srv.model_state.model_name="move_button_side"
    srv.model_state.pose = TURN_BUTTON_POSE_SIDE
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()
def show_move_buttons(req = None):
    srv = SetModelStateRequest()
    srv.model_state.model_name="move_button_main"
    srv.model_state.pose = MOVE_BUTTON_POSE_MAIN
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    srv.model_state.model_name="move_button_side"
    srv.model_state.pose = MOVE_BUTTON_POSE_SIDE
    srv.model_state.reference_frame = "world"
    g_set_model_state_client(srv)
    return EmptyResponse()
def hide_move_buttons(req = None):
    srv = SetModelStateRequest()
    srv.model_state.pose = g_far_zone_pose
    srv.model_state.reference_frame = "world"
    srv.model_state.model_name="move_button_main"
    g_set_model_state_client(srv)
    srv.model_state.model_name="move_button_side"
    g_set_model_state_client(srv)
    return EmptyResponse()

Q1_pose = Pose(Point(1.2, 0.53, 1.33), quaternion_msgs_from_rpy(-0.6, 0, 1.57))
Q2_pose = Pose(Point(1.2, 0.68, 1.33), quaternion_msgs_from_rpy(-0.6, 0, 1.57))
Q3_pose = Pose(Point(1.2, 0.83, 1.33), quaternion_msgs_from_rpy(-0.6, 0, 1.57))
Q4_pose = Pose(Point(1.3, 0.67, 1.22), quaternion_msgs_from_rpy(-0.6, 0, 1.57))
Q5_pose = Pose(Point(1.3, 0.82, 1.22), quaternion_msgs_from_rpy(-0.6, 0, 1.57))
def set_question_buttons(req = None):

    if req==None:
        req = SetQuestionButtonsRequest()
        req.q1 = True
        req.q2 = True
        req.q3 = True
        req.q4 = True
        req.q5 = True
    
    srv = SetModelStateRequest()
    srv.model_state.reference_frame = "world"

    srv.model_state.model_name="q1"
    srv.model_state.pose = Q1_pose if req.q1 else g_far_zone_pose
    g_set_model_state_client(srv)
    
    srv.model_state.model_name="q2"
    srv.model_state.pose = Q2_pose if req.q2 else g_far_zone_pose
    g_set_model_state_client(srv)
    
    srv.model_state.model_name="q3"
    srv.model_state.pose = Q3_pose if req.q3 else g_far_zone_pose
    g_set_model_state_client(srv)
    
    srv.model_state.model_name="q4"
    srv.model_state.pose = Q4_pose if req.q4 else g_far_zone_pose
    g_set_model_state_client(srv)
    
    srv.model_state.model_name="q5"
    srv.model_state.pose = Q5_pose if req.q5 else g_far_zone_pose
    g_set_model_state_client(srv)

    return SetQuestionButtonsResponse()

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

    # Reset all zones
    for z in g_zones:
        z.current_action_id = -10

    # Update pass zone
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

    for i,ha in enumerate(g_vha.valid_human_actions):

        # Questions
        if ha[:len("communicate_if_cube_can_be_put")] == "communicate_if_cube_can_be_put":
            n_box = int(ha[ha.find('box_')+len('box_')])
            if n_box==1:
                g_z_q1.current_action_id = i+1
            elif n_box==2:
                g_z_q2.current_action_id = i+1
            elif n_box==3:
                g_z_q3.current_action_id = i+1
            elif n_box==4:
                g_z_q4.current_action_id = i+1
            elif n_box==5:
                g_z_q5.current_action_id = i+1
            continue

        # Regular actions
        found = False
        for z in g_zones:
            for z_a in z.valid_actions:
                if z_a == ha[:len(z_a)]: # ha starts with z_a
                    z.current_action_id = i+1
                    found = True
                    # If turn or move actions, show buttons
                    if z_a == "change_focus_towards":
                        show_turn_buttons()
                    elif z_a =="move_to_table":
                        show_move_buttons()
                    break
            # If found corresponding zone, stop current loop
            if found:
                break


    lines = []
    srv_set_q_buttons = SetQuestionButtonsRequest()
    srv_set_q_buttons.q1 = False
    srv_set_q_buttons.q2 = False
    srv_set_q_buttons.q3 = False
    srv_set_q_buttons.q4 = False
    srv_set_q_buttons.q5 = False
    if g_z_q1.current_action_id != -10:
        lines.append("1- Is box 1 empty?")
        srv_set_q_buttons.q1 = True
    if g_z_q2.current_action_id != -10:
        lines.append("2- Is box 2 empty?")
        srv_set_q_buttons.q2 = True
    if g_z_q3.current_action_id != -10:
        lines.append("3- Is box 3 empty?")
        srv_set_q_buttons.q3 = True
    if g_z_q4.current_action_id != -10:
        lines.append("4- Is box 4 empty?")
        srv_set_q_buttons.q4 = True
    if g_z_q5.current_action_id != -10:
        lines.append("5- Is box 5 empty?")
        srv_set_q_buttons.q5 = True

    set_question_buttons(srv_set_q_buttons)
    if len(lines):
        # print in shell
        rospy.logwarn("Questions:")
        for l in lines:
            rospy.logwarn(l)

        # in prompt window
        msg_questions = String()
        msg_questions.data = "Questions:\n"
        for l in lines:
            msg_questions.data += f"{l}\n"
        g_prompt_pub.publish(msg_questions)

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
                disable_zones()
                # hide_can_click_indicator()
                rospy.loginfo("done hiding zones")
            else:
                rospy.loginfo("timeout canceled")

decision_sent = False
g_last_click = Click(Point(-1,-1,-1),-1) # type: Click
def mouse_pressed_cb(msg: Point):
    global g_last_click
    g_last_click = Click(msg, time.time())
    rospy.loginfo(f"click: {msg.x},{msg.y}")
def reset_last_click(req = None):
    global g_last_click
    g_last_click = Click(Point(-1,-1,-1),-1)
    rospy.loginfo("click reset")
    return EmptyResponse()

def TO_reached_cb(msg: EmptyM):
    disable_zones()
    # hide_can_click_indicator()

##########
## MAIN ##
##########

def main():
    global g_vha, g_vha_received, g_step_over, g_timeout_max, g_best_human_action, g_human_choice_pub, g_set_model_state_client, g_start_human_action_prox
    global g_prompt_button_pressed_pub, g_prompt_pub
    global decision_sent
    global AUTO_PASS

    rospy.init_node('mouse_human', log_level=rospy.INFO)

    human_vha_sub = rospy.Subscriber("hmi_vha", VHA, incoming_vha_cb, queue_size=1)
    click_sub = rospy.Subscriber("/mouse_pressed_pose", Point, mouse_pressed_cb, queue_size=1)
    TO_reached_sub = rospy.Subscriber("/hmi_timeout_reached", EmptyM, TO_reached_cb, queue_size=1)
    reset_auto_pass_sub = rospy.Subscriber("/reset_auto_pass", EmptyM, reset_auto_pass_cb, queue_size=1)

    timeout_max_service = rospy.Service("hmi_timeout_max", Int, lambda req: IntResponse())
    r_idle_service = rospy.Service("hmi_r_idle", SetBool, lambda req: SetBoolResponse())
    disable_zones_service = rospy.Service("disbale_zones", EmptyS, disable_zones)
    show_prompt_button_service = rospy.Service("show_prompt_button", EmptyS, show_prompt_button)
    hide_prompt_button_service = rospy.Service("hide_prompt_button", EmptyS, hide_prompt_button)
    show_tuto_zones_service = rospy.Service("show_tuto_zones", EmptyS, show_tuto_zones)
    hide_tuto_zones_service = rospy.Service("hide_tuto_zones", EmptyS, hide_tuto_zones)
    show_auto_pass_indicator_service = rospy.Service("show_auto_pass_indicator", EmptyS, show_auto_pass_indicator)
    hide_auto_pass_indicator_service = rospy.Service("hide_auto_pass_indicator", EmptyS, hide_auto_pass_indicator)
    show_turn_buttons_service = rospy.Service("show_turn_buttons", EmptyS, show_turn_buttons)
    show_move_buttons_service = rospy.Service("show_move_buttons", EmptyS, show_move_buttons)
    hide_move_buttons_service = rospy.Service("hide_move_buttons", EmptyS, hide_move_buttons)
    set_question_buttons_service = rospy.Service("set_question_buttons", SetQuestionButtons, set_question_buttons)

    g_prompt_pub = rospy.Publisher("/simu_prompt", String, queue_size=1)
    

    auto_pass_pub = rospy.Publisher("/auto_pass_state", Bool, queue_size=1)


    reset_last_click_service = rospy.Service("reset_last_click", EmptyS, reset_last_click)

    g_prompt_button_pressed_pub = rospy.Publisher('/prompt_button_pressed', EmptyM, queue_size=1)


    rospy.wait_for_service("/gazebo/set_model_state")
    g_set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)


    # SPAWNING #
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    # spawn prompt button
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/prompt_button.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"prompt_button", sdff, "", g_far_zone_pose, "world")
    print(f"prompt_button spawned")

    # spawn move_button
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/move_button.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"move_button_main", sdff, "", g_far_zone_pose, "world")
    print(f"move_button main spawned")
    spawn_model_prox(f"move_button_side", sdff, "", g_far_zone_pose, "world")
    print(f"move_button side spawned")

    # spawn question buttons
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/q1.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"q1", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/q2.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"q2", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/q3.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"q3", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/q4.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"q4", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/q5.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"q5", sdff, "", g_far_zone_pose, "world")

    # spawn tuto zones
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/t1.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"t1", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/t2.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"t2", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/t3.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"t3", sdff, "", g_far_zone_pose, "world")
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/t4.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"t4", sdff, "", g_far_zone_pose, "world")

    # spawn auto_click indicator
    f = open(f'/home/sshekhar/exec_simulator/src/simulator/worlds/auto_pass_indicator.sdf','r')
    sdff = f.read()
    f.close()
    spawn_model_prox(f"auto_pass_indicator", sdff, "", g_far_zone_pose, "world")

    ##################

    started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
    rospy.loginfo("HMI ready")

    rospy.wait_for_service("start_human_action")
    g_start_human_action_prox = rospy.ServiceProxy("start_human_action", Int)

    AUTO_PASS = False

    while not rospy.is_shutdown():

        if time.time()-g_last_click.time>2.0:
            continue

        # PROMPT BUTTON
        if g_prompt_button_shown:
            if g_prompt_button_zone.clickIn(g_last_click):
                rospy.loginfo("prompt button pressed !")
                g_prompt_button_pressed_pub.publish(EmptyM())
                hide_prompt_button()
                reset_last_click()

        else:
            # ACTION ZONES
            for z in g_zones:
                if z.isActive() and z.clickIn(g_last_click):
                    decision_sent = True
                    rospy.loginfo(f"zone clicked: {z}")
                    g_start_human_action_prox(z.current_action_id)
                    disable_zones()
                    rospy.loginfo("human decision sent")
                    reset_last_click()
                    break

        time.sleep(0.01)
            
if __name__ == "__main__":
    main()


