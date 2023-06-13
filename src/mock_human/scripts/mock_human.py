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

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')

g_vha = None
g_vha_received = False
g_step_over = False
g_timeout_max = 0
g_best_human_action = 0

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
        raise Exception("HumanPolicy method not overrided!")
    def compliant_delay(self):
        raise Exception("HumanPolicy method not overrided!")
    def make_start_choice(self):
        raise Exception("HumanPolicy method not overrided!")
    def make_compliant_choice(self):
        raise Exception("HumanPolicy method not overrided!")
    def select_action(self):
        raise Exception("HumanPolicy method not overrided!")

class ActingPolicy(HumanPolicy):
    def __init__(self) -> None:
        self.r_idle = False

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
    
    def wait_robot_start_acting(self):
        if not self.r_idle:
            global g_vha_received
            while not rospy.is_shutdown() and not g_vha_received:
                rospy.sleep(0.05)
            g_vha_received = False
    
class RandomPolicy(HumanPolicy):
    def __init__(self) -> None:
        self.choice = None
        self.r_idle = False

    def start_delay(self):
        log_human("S_SDELAY")
        self.choice = None

        # DEFINE duration of delay
        duration = random.uniform(0.0, 0.9*g_timeout_max)

        # WAIT with progress bar
        bar = IncrementalBar('Start Delay', max=duration, suffix='%(index).1f/%(max).1fs')
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-start_time).to_sec() < duration:
            rospy.sleep(0.01)
            bar.goto((rospy.Time.now()-start_time).to_sec())
        bar.finish()

        log_human("E_SDELAY")

    def compliant_delay(self):
        log_human("S_CDELAY")

        # DEFINE duration of delay
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
        log_human("E_CDELAY")
        return False
    
    def make_start_choice(self):
        self.choice = random.choice( range(0, len(g_vha.valid_human_actions)+1) )
        if self.choice == 0:
            if self.r_idle:
                return HumanChoice.PASS
            else:
                return random.choice([HumanChoice.PASS, HumanChoice.WAIT])
        else:
            return HumanChoice.ACT
    
    def make_compliant_choice(self):
        self.choice = random.choice( range(0, len(g_vha.valid_human_actions)+1) )
        if self.choice == 0 and not self.r_idle:
            return HumanChoice.WAIT
        else:
            return HumanChoice.ACT

    def select_action(self):
        return self.choice
    
    def wait_robot_start_acting(self):
        if not self.r_idle:
            global g_vha_received
            while not rospy.is_shutdown() and not g_vha_received:
                rospy.sleep(0.05)
            g_vha_received = False

class LazyPolicy(HumanPolicy):
    def __init__(self) -> None:
        self.r_idle = False

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
        self.choice = random.choice( range(0, len(g_vha.valid_human_actions)+1) )
        if self.choice != 0 and g_vha.valid_human_actions[self.choice-1] in ["pick('p', 'H')", "place('p', 'l3')"]:
            return HumanChoice.ACT
        else:
            self.choice = 0
            return random.choice([HumanChoice.PASS, HumanChoice.WAIT])
    
    def make_compliant_choice(self):
        self.choice = random.choice( range(0, len(g_vha.valid_human_actions)+1) )
        if self.choice != 0 and g_vha.valid_human_actions[self.choice-1] in ["pick('p', 'H')", "place('p', 'l3')"]:
            return HumanChoice.ACT
        else:
            self.choice = 0
            return random.choice([HumanChoice.PASS, HumanChoice.WAIT])

    def select_action(self):
        return self.choice
    
    def wait_robot_start_acting(self):
        if not self.r_idle:
            global g_vha_received
            while not rospy.is_shutdown() and not g_vha_received:
                rospy.sleep(0.05)
            g_vha_received = False

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

def log_human(name):
    msg = EventLog()
    msg.name = name
    msg.timestamp = rospy.get_time()
    g_h_event_log_pub.publish(msg)

def set_r_idle(req):
    global g_h_policy
    g_h_policy.r_idle = req.data
    return SetBoolResponse()

##########
## MAIN ##
##########

g_h_event_log_pub = None
g_h_policy = None # type: HumanPolicy | None
def main():
    global g_vha, g_vha_received, g_step_over, g_timeout_max, g_best_human_action, g_h_event_log_pub, g_h_policy
    sys.setrecursionlimit(100000)

    # domain_name, solution_tree, begin_step = load_solution()
    # initDomain()

    # Init Seed
    seed = random.randrange(sys.maxsize)
    random.seed(seed)
    lg.debug(f"\nSeed was: {seed}")

    rospy.init_node('mock_human', log_level=rospy.INFO)
    human_choice_pub = rospy.Publisher("human_choice", Int32, queue_size=1)
    human_vha_sub = rospy.Subscriber("hmi_vha", VHA, incoming_vha_cb, queue_size=1)
    step_over_sub = rospy.Subscriber("step_over", EmptyM, step_over_cb, queue_size=1)
    best_human_action_sub = rospy.Subscriber("mock_best_human_action", Int32, best_human_action_cb, queue_size=1)

    g_h_event_log_pub = rospy.Publisher("/h_event_log", EventLog, queue_size=10)

    started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
    timeout_max_service = rospy.Service("hmi_timeout_max", Int, set_timeout_max)
    idle_service = rospy.Service("hmi_r_idle", SetBool, set_r_idle)



    h_choice_msg = Int32()

    # Define human policy
    # g_h_policy = ActingPolicy()
    g_h_policy = RandomPolicy()
    # g_h_policy = LazyPolicy()

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
        rospy.loginfo("New step, waiting to receive vha...")
        while not rospy.is_shutdown() and not g_vha_received:
            rospy.sleep(0.05)
        g_vha_received = False
        g_step_over = False

        # Random delay [0, To]
        g_h_policy.start_delay()

        # According to policy, either WAIT, PASS, ACT
        choice = g_h_policy.make_start_choice()

        ## ACT FIRST ##
        if HumanChoice.ACT == choice:
            # Select action among VHA and send
            h_choice_msg.data = g_h_policy.select_action()
            s = f"ACT FIRST {h_choice_msg.data}: "
            if h_choice_msg.data==-1:
                s += "PASS"
            else:
                s += f"{g_vha.valid_human_actions[h_choice_msg.data-1]}"
            rospy.loginfo(s)
            human_choice_pub.publish(h_choice_msg)
        else:
            if HumanChoice.PASS == choice:
                # Send pass choice
                rospy.loginfo("PASS")
                h_choice_msg.data = -1
                human_choice_pub.publish(h_choice_msg)
            elif HumanChoice.WAIT == choice:
                rospy.loginfo("WAIT")
                while not rospy.is_shutdown() and not g_vha_received:
                    rospy.sleep(0.05)

            # Random delay [0, ?] (is interrupted by step over event => Continue)
            if g_h_policy.compliant_delay():
                # True: has been interrupted
                g_vha_received = False
                continue
            else:
                g_h_policy.wait_robot_start_acting()

                choice = g_h_policy.make_compliant_choice()

                ## ACT AFTER ##
                if HumanChoice.ACT == choice:
                    # Select action among VHA and send
                    h_choice_msg.data = g_h_policy.select_action()
                    s = f"ACT AFTER {h_choice_msg.data}: "
                    if h_choice_msg.data==-1:
                        s += "PASS"
                    else:
                        s += f"{g_vha.valid_human_actions[h_choice_msg.data-1]}"
                    rospy.loginfo(s)
                    human_choice_pub.publish(h_choice_msg)
                    pass
                else:
                    ## PASSIVE ##
                    if HumanChoice.PASS == choice:
                        rospy.loginfo("PASS")
                        # Send pass choice
                        h_choice_msg.data = -1
                        human_choice_pub.publish(h_choice_msg)
                    elif HumanChoice.WAIT == choice:
                        rospy.loginfo("WAIT")
                        # Wait for step_over
                        while not rospy.is_shutdown() and not g_step_over:
                            rospy.sleep(0.05)

    
if __name__ == "__main__":
    main()


