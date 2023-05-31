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
import importlib

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')

#################
## HumanPolicy ##
#################
class HumanChoice(Enum):
    WAIT = 0
    PASS = 1
    ACT = 2
class HumanPolicy:
    def __init__(self) -> None:
        pass

    def start_delay(self):
        # DEFINE duration of delay
        # duration = 2.0
        duration = random.uniform(0.0, 0.9*g_timeout_max)

        # WAIT with progress bar
        bar = IncrementalBar('Start Delay', max=duration, suffix='%(index).1f/%(max).1fs')
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-start_time).to_sec() < duration:
            rospy.sleep(0.01)
            bar.goto((rospy.Time.now()-start_time).to_sec())
        bar.finish()

    def compliant_delay(self):
        # DEFINE duration of delay
        # duration = 3.0
        duration = random.uniform(1.0, 3.0)

        # WAIT with progress bar
        bar = IncrementalBar('Compliant Delay', max=duration, suffix='%(index).1f/%(max).1fs')
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-start_time).to_sec() < duration:
            rospy.sleep(0.01)
            bar.goto((rospy.Time.now()-start_time).to_sec())
            if g_step_over:
                bar.finish()
                return True
        bar.finish()
        return False
    
    def make_start_choice(self):
        possible_choices = [HumanChoice.ACT, HumanChoice.PASS, HumanChoice.WAIT]
        return random.choices(possible_choices, weights=(70, 15, 15), k=1)[0]
    
    def make_compliant_choice(self):
        possible_choices = [HumanChoice.ACT, HumanChoice.PASS, HumanChoice.WAIT]
        return random.choices(possible_choices, weights=(70, 15, 15), k=1)[0]

    def select_action(self):
        # i = random.randint(1,len(g_vha))
        i = g_best_human_action
        return i
    

#########
## ROS ##
#########
def incoming_vha_cb(msg):
    global g_vha, g_vha_received
    g_vha = msg
    g_vha_received = True

def step_over_cb(msg):
    global g_step_over
    g_step_over = True

def set_timeout_max(req):
    global g_timeout_max
    g_timeout_max = req.data
    return IntResponse()

def best_human_action_cb(msg):
    global g_best_human_action
    g_best_human_action = msg.data

##########
## MAIN ##
##########
g_vha = None
g_vha_received = False
g_step_over = False
g_timeout_max = 0
g_best_human_action = 0
def main():
    global g_vha, g_vha_received, g_step_over, g_timeout_max, g_best_human_action
    sys.setrecursionlimit(100000)

    # domain_name, solution_tree, begin_step = load_solution()
    # initDomain()

    # Init Seed
    seed = random.randrange(sys.maxsize)
    random.seed(seed)
    lg.debug(f"\nSeed was: {seed}")

    rospy.init_node('mock_human')
    human_choice_pub = rospy.Publisher("human_choice", Int32, queue_size=1)
    human_vha_sub = rospy.Subscriber("hmi_vha", VHA, incoming_vha_cb, queue_size=1)
    step_over_sub = rospy.Subscriber("step_over", EmptyM, step_over_cb, queue_size=1)
    best_human_action_sub = rospy.Subscriber("mock_best_human_action", Int32, best_human_action_cb, queue_size=1)

    started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
    timeout_max_service = rospy.Service("hmi_timeout_max", Int, set_timeout_max)


    h_choice_msg = Int32()

    # Define human policy
    h_policy = HumanPolicy()

    # LOOP
        # Wait receive available actions
        # Random delay [0, To]
        # Make start choice: According to policy, either WAIT, PASS, ACT
        # if active
            # Select and send action among available actions
        # if passive
            # if PASS send pass choice
            # if WAIT wait until To
            # Random delay [0, ?] (is interrupted by step over event => Continue)
            # Make compliant choice:  Accoding to policy, either WAIT, PASS, ACT

    # LOOP
    while not rospy.is_shutdown():
        # Wait receive available actions
        print("New step, waiting to receive vha...")
        while not rospy.is_shutdown() and not g_vha_received:
            rospy.sleep(0.05)
        g_vha_received = False
        g_step_over = False

        # Random delay [0, To]
        h_policy.start_delay()

        # According to policy, either WAIT, PASS, ACT
        choice = h_policy.make_start_choice()

        ## ACT FIRST ##
        if HumanChoice.ACT == choice:
            # Select action among VHA and send
            h_choice_msg.data = h_policy.select_action()
            print(f"ACT {h_choice_msg.data}: ", end="")
            if h_choice_msg.data==-1:
                print(f"PASS")
            else:
                print(f"{g_vha.valid_human_actions[h_choice_msg.data-1]}")
            human_choice_pub.publish(h_choice_msg)
        else:
            if HumanChoice.PASS == choice:
                # Send pass choice
                print("PASS")
                h_choice_msg.data = -1
                human_choice_pub.publish(h_choice_msg)
            elif HumanChoice.WAIT == choice:
                print("WAIT")
                while not rospy.is_shutdown() and not g_vha_received:
                    rospy.sleep(0.05)

            # Random delay [0, ?] (is interrupted by step over event => Continue)
            if h_policy.compliant_delay():
                # True: has been interrupted
                g_vha_received = False
                continue
            else:
                # Be sure we received a new set of human actions
                while not rospy.is_shutdown() and not g_vha_received:
                    rospy.sleep(0.05)
                g_vha_received = False

                choice = h_policy.make_compliant_choice()

                ## ACT AFTER ##
                if HumanChoice.ACT == choice:
                    # Select action among VHA and send
                    h_choice_msg.data = h_policy.select_action()
                    print(f"ACT {h_choice_msg.data}: ", end="")
                    if h_choice_msg.data==-1:
                        print(f"PASS")
                    else:
                        print(f"{g_vha.valid_human_actions[h_choice_msg.data-1]}")
                    human_choice_pub.publish(h_choice_msg)
                    pass
                else:
                    ## PASSIVE ##
                    if HumanChoice.PASS == choice:
                        print("PASS")
                        # Send pass choice
                        h_choice_msg.data = -1
                        human_choice_pub.publish(h_choice_msg)
                    elif HumanChoice.WAIT == choice:
                        print("WAIT")
                        # Wait for step_over
                        while not rospy.is_shutdown() and not g_step_over:
                            rospy.sleep(0.05)

    
if __name__ == "__main__":
    main()


