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
from sim_msgs.srv import Int, IntResponse
from std_srvs.srv import Empty as EmptyS
from std_srvs.srv import EmptyResponse
from progress.bar import IncrementalBar
from sim_msgs.msg import EventLog
import importlib
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Point

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')


class Zone:
    def __init__(self, id, x1, y1, x2, y2) -> None:
        self.id = id
        # Right-Upper corner (x1,y1)
        self.x1 = x1
        self.y1 = y1
        # Left-Lower corner (x2,y2)
        self.x2 = x2
        self.y2 = y2

    def __repr__(self) -> str:
        return f"{self.id}-({self.x1},{self.y1})-({self.x2},{self.y2})"

g_vha = VHA()
g_vha_received = False
g_step_over = False
g_timeout_max = 0
g_best_human_action = 0

g_zones = []

g_zones.append( Zone(0,904,361,1127,552))
g_zones.append( Zone(1,515,447,616,550))
g_zones.append( Zone(2,269,630,382,740))
g_zones.append( Zone(3,406,637,505,747))
g_zones.append( Zone(4,553,643,791,745))
g_zones.append( Zone(5,1113,657,1282,850))



def isInZone(pose: Point, zone: Zone):
    return pose.x>=zone.x1 and pose.x<=zone.x2 and pose.y>=zone.y1 and pose.y<=zone.y2

#########
## ROS ##
#########
def incoming_vha_cb(msg):
    global g_vha, g_vha_received
    print("vha received")
    print(msg)
    g_vha = msg
    g_vha_received = True

def mouse_pressed_cb(msg: Point):
    print("oboib")

    zone_clicked = None
    for z in g_zones:
        if isInZone(msg, z):
            zone_clicked = z
            break
    print(f"zone clicked: {zone_clicked}")

    if zone_clicked!=None:
        if      0 == zone_clicked.id:
            for i,ha in enumerate(g_vha.valid_human_actions):
                action_name = "place"
                if ha[:len(action_name)] == action_name:
                    g_human_choice_pub.publish(Int32(i+1))
                    print("published")
                    break
        elif    1 == zone_clicked.id:
            for i,ha in enumerate(g_vha.valid_human_actions):
                action_name = "pick('y', 'C')"
                if ha[:len(action_name)] == action_name:
                    g_human_choice_pub.publish(Int32(i+1))
                    print("published")
                    break
        elif    2 == zone_clicked.id:
            for i,ha in enumerate(g_vha.valid_human_actions):
                action_name = "pick('r', 'H')"
                if ha[:len(action_name)] == action_name:
                    g_human_choice_pub.publish(Int32(i+1))
                    print("published")
                    break
        elif    3 == zone_clicked.id:
            for i,ha in enumerate(g_vha.valid_human_actions):
                action_name = "pick('b', 'H')"
                if ha[:len(action_name)] == action_name:
                    g_human_choice_pub.publish(Int32(i+1))
                    print("published")
                    break
        elif    4 == zone_clicked.id:
            for i,ha in enumerate(g_vha.valid_human_actions):
                action_name = "pick('p', 'H')"
                if ha[:len(action_name)] == action_name:
                    g_human_choice_pub.publish(Int32(i+1))
                    print("published")
                    break
        elif    5 == zone_clicked.id:
            g_human_choice_pub.publish(Int32(-1))
        elif    6 == zone_clicked.id:
            pass
        elif    7 == zone_clicked.id:
            pass
        elif    8 == zone_clicked.id:
            pass
        elif    9 == zone_clicked.id:
            pass
        elif    10 == zone_clicked.id:
            pass

##########
## MAIN ##
##########

def main():
    global g_vha, g_vha_received, g_step_over, g_timeout_max, g_best_human_action, g_human_choice_pub

    rospy.init_node('mouse_human', log_level=rospy.INFO)
    g_human_choice_pub = rospy.Publisher("human_choice", Int32, queue_size=1)

    human_vha_sub = rospy.Subscriber("hmi_vha", VHA, incoming_vha_cb, queue_size=1)
    click_sub = rospy.Subscriber("/mouse_pressed_pose", Point, mouse_pressed_cb, queue_size=1)

    started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
    timeout_max_service = rospy.Service("hmi_timeout_max", Int, lambda req: IntResponse())
    r_idle_service = rospy.Service("hmi_r_idle", SetBool, lambda req: SetBoolResponse())

    rospy.spin()

if __name__ == "__main__":
    main()


