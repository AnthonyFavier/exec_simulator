#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Dict, List, Tuple
from copy import deepcopy
import random
import pickle
import dill
import sys
import importlib
import rospy
import logging as lg
import logging.config
from enum import Enum
import matplotlib.pyplot as plt
from progress.bar import IncrementalBar
from std_msgs.msg import Int32, Bool
from std_msgs.msg import Empty as EmptyM
from std_srvs.srv import Empty as EmptyS
from std_srvs.srv import SetBool
from sim_msgs.msg import Action, VHA
from sim_msgs.msg import EventLog
from sim_msgs.srv import Int, IntResponse
from sim_msgs.msg import Signal

class IdResult(Enum):
    NOT_NEEDED=0
    FAILED=1

DEBUG = False
INPUT = False
########
# DEBUG = True
# INPUT = True

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)
import ConcurrentModule as ConM
import CommonModule as CM

# Dynamically loads domain file
g_domain_name = rospy.get_param("/domain_name")
spec = importlib.util.spec_from_file_location(g_domain_name, path+g_domain_name+".py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
sys.modules["domain"] = module
from domain import *

step_over = True

class WrongException(Exception):
    pass

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')

#############
## LOADING ##
#############
begin_step = None
def load_solution():
    """
    Loads the previously produced solution.
    The domain name is retreived and returned and as well as the solution tree and the initial step.
    """
    dom_n_sol = dill.load(open(CM.path + "dom_n_sol.p", "rb"))

    domain_name = dom_n_sol[0]
    solution_tree = dom_n_sol[1]
    init_step = solution_tree[0]

    if domain_name!=g_domain_name:
        raise Exception("Mismatching domain names!")

    return domain_name, solution_tree, init_step

def set_choices(init_step,r_criteria,h_criteria):

    final_leaves = init_step.get_final_leaves()

    r_ranked_leaves = ConM.sorting_branches(final_leaves, r_criteria, is_robot=True) #type: List[ConM.Step]
    h_ranked_leaves = ConM.sorting_branches(final_leaves, h_criteria, is_robot=False) #type: List[ConM.Step]

    ConM.update_robot_choices(init_step)
    ConM.update_human_choices(init_step)

    return r_ranked_leaves, h_ranked_leaves

###############
## EXECUTION ##
###############
def execution_simulation(begin_step: ConM.Step, r_pref, h_pref, r_ranked_leaves, h_ranked_leaves):
    """
    Main algorithm 
    """
    curr_step = get_first_step(begin_step)
    nb_of_degradation = 0
    r_is_idle = False
    while not exec_over(curr_step) and not rospy.is_shutdown():

        MOCK_update_human_actions(curr_step)

        if curr_step.isRInactive():
            send_next_step_idle()
            go_idle_pose(r_is_idle)
            r_is_idle = True
            RA = pick_valid_passive(curr_step)
            wait_human_start_acting(curr_step)

        else:
            go_home_pose(r_is_idle)
            r_is_idle = False
            send_next_step()
            wait_human_choice(curr_step)
            MOCK_save_best_reachable_solution_for_human(curr_step)

            ## 1 & 2 & 3 ##
            if human_active(): 
                ## 1 & 2 ##
                if ID_needed(curr_step): 
                    result_id = MOCK_run_id_phase(curr_step)
                    ## 1 ##
                    if ID_successful(result_id): 
                        RA = pick_best_valid_RA(curr_step, result_id)
                    ## 2 ##
                    else: 
                        RA = pick_valid_passive(curr_step)
                ## 3 ##
                else: 
                    lg.debug("ID not needed.")
                    RA = pick_best_RA(curr_step)
            ## 4 & 5 ##
            else: 
                RA = pick_best_RA_H_passive(curr_step)

            start_execute_RA(RA)
            MOCK_update_hmi_human_choices(curr_step, RA)
                  
        wait_step_end()
        
        HA = MOCK_assess_human_action(RA)

        # Check Passive Step
        if RA.is_passive() and HA.is_passive():
            # Repeat current step
            reset_human()
            rospy.sleep(0.1)
        else:
            curr_step = get_next_step(curr_step, HA, RA)
            MOCK_save_best_reachable_solution_for_human_after_robot_choice(curr_step)
            if MOCK_robot_has_degraded_human_best_solution():
                nb_of_degradation +=1
                if nb_of_degradation >= 1:
                    r_ranked_leaves, h_ranked_leaves = adjust_robot_preferences(begin_step,h_pref,h_pref)

            reset_human()
            rospy.sleep(0.1)

    log_event("OVER")
    lg.info(f"END => {curr_step}")
    print(f"END => {curr_step}")
    g_hmi_finish_pub.publish(EmptyM())
    # return int(curr_step.id)
    return int(curr_step.id), curr_step.get_f_leaf().branch_rank_r, curr_step.get_f_leaf().branch_rank_h, r_ranked_leaves, h_ranked_leaves

#############
## SIGNALS ##
#############

def send_next_step_idle():
    log_event("NS_IDLE")
    g_robot_next_step_idle_pub.publish(EmptyM())
    
def send_next_step():
    log_event("NS")
    g_robot_next_step_pub.publish(EmptyM())


#########################
## MOCK Human behavior ##
#########################
HC = None
g_p_lrde = None
g_p_lrdo = None

g_best_reachable_human_solution = None
def MOCK_save_best_reachable_solution_for_human(step: ConM.Step):
    if not HUMAN_UPDATING:
        return
    global g_best_reachable_human_solution
    s = step

    while not s.is_final:
        s = s.best_human_pair.next[0].get_in_step()
    
    g_best_reachable_human_solution = s

g_best_reachable_human_solution_after_robot_choice = None
def MOCK_save_best_reachable_solution_for_human_after_robot_choice(step: ConM.Step):
    if not HUMAN_UPDATING:
        return
    global g_best_reachable_human_solution_after_robot_choice
    s = step

    while not s.is_final:
        s = s.best_human_pair.next[0].get_in_step()
    
    g_best_reachable_human_solution_after_robot_choice = s

def MOCK_robot_has_degraded_human_best_solution():
    if not HUMAN_UPDATING:
        return False
    rank_before = g_best_reachable_human_solution.get_f_leaf().branch_rank_h
    rank_after = g_best_reachable_human_solution_after_robot_choice.get_f_leaf().branch_rank_h

    lg.debug(f"#{rank_before} -> #{rank_after}")

    return rank_after>rank_before

def adjust_robot_preferences(begin_step,r_pref,h_pref):
    r_ranked_leaves, h_ranked_leaves = set_choices(begin_step,r_pref,h_pref)
    lg.debug("ROBOT PREFERENCES ALIGNED !!!!")
    return (r_ranked_leaves, h_ranked_leaves)

def MOCK_update_hmi_human_choices(curr_step, RA):
    if not human_active():
        compliant_pairs = find_compliant_pairs_with_RA(curr_step, RA)
        compliant_human_actions = [p.human_action for p in compliant_pairs]
        update_vha(compliant_human_actions, False)

        # find best human action in compliant actions
        # find best pair
        best_rank_i = 0
        best_rank_v = compliant_pairs[best_rank_i].best_rank_h
        for i,p in enumerate(compliant_pairs[1:]):
            if p.best_rank_h < best_rank_v:
                best_rank_i = i
                best_rank_v = p.best_rank_h
        # find id of corresponding best action
        best_ha = Int32()
        if compliant_pairs[best_rank_i].human_action.is_passive():
            best_ha.data = -1
        else:
            for i,ho in enumerate(curr_step.human_options):
                if CM.Action.are_similar( ho.human_action, compliant_pairs[best_rank_i].human_action ):
                    best_ha.data = i+1
                    break
        g_best_human_action.publish(best_ha)

def find_compliant_pairs_with_RA(curr_step: ConM.Step, RA):
    compliant_pairs = []
    for pair in curr_step.get_pairs():
        if CM.Action.are_similar(pair.robot_action, RA):
            compliant_pairs.append(pair)
    return compliant_pairs

##########################
## MOCK Robot execution ##
##########################

def MOCK_run_id_phase(step: ConM.Step):
    """
    Simulate the identification phase.
    Wait ID_DELAY then identify human action with a P_SUCCESS_ID chance. 
    """
    log_event("R_S_ID")

    rospy.loginfo("Start ID phase...")
    rospy.sleep(ID_DELAY)
    if random.random()<P_SUCCESS_ID_PHASE:
        rospy.loginfo("ID Success")
        id_result = HC
    else:
        rospy.loginfo("ID failed...")
        id_result = None
    
    log_event("R_E_ID")
    
    return id_result

def MOCK_assess_human_action(RA):
    global HC
    # If Human didn't choose, we try to find passive human action
    log_event("R_S_ASSESS")

    if HC==None:
        for ha in g_possible_human_actions:
            if ha.is_passive():
                HC = ha
                break
        # If didn't find passive HA
        if HC==None: 
            rospy.loginfo("Inactive step...")
            HC = default_human_passive_action
    else:
        rospy.loginfo(f"Assessed Human action: {HC}")

    rospy.sleep(ASSESS_DELAY)
    log_event("R_E_ASSESS")
    
    return HC

def get_first_step(begin_step: ConM.Step):
    if len(begin_step.children)!=1:
        raise Exception("begin_step should only have 1 child.")
    first_step = begin_step.children[0]
    return first_step

def get_agents_before_step(step: ConM.Step):
    return step.from_pair.end_agents

def exec_over(step):
    return step.is_final

def go_idle_pose(r_is_idle):
    if not r_is_idle:
        g_hmi_r_idle_client.call(True)
        g_go_idle_pose_client.call()

def go_home_pose(r_is_idle):
    if r_is_idle:
        g_hmi_r_idle_client.call(False)
        g_go_home_pose_client.call()

def MOCK_update_human_actions(step: ConM.Step):
    """
    Updates g_possible_human_actions, send them to Human, send best_human_action id
    """
    global g_possible_human_actions
    # i.e. wait for human to start acting or timeout reached
    rospy.loginfo("Current step:\n"+CM.str_agents(get_agents_before_step(step)))
    rospy.loginfo("\n" + step.str())

    g_possible_human_actions = [ho.human_action for ho in step.human_options]

    update_vha(g_possible_human_actions, True)

    # Find best human action id, sent to hmi mock
    best_ha = Int32()
    if step.best_human_pair.human_action.is_passive():
        best_ha.data = -1
    else:
        for i,ho in enumerate(step.human_options):
            if CM.Action.are_similar(ho.human_action, step.best_human_pair.human_action):
                best_ha.data = i+1
                break
    g_best_human_action.publish(best_ha)

def wait_human_start_acting(step: ConM.Step):
    global HC, step_over, g_previous_elapsed

    rospy.loginfo("Waiting for human to act...")
    while not rospy.is_shutdown():
        if HC!=None and not HC.is_passive():
            break
        rospy.sleep(0.1)

    rospy.sleep(WAIT_START_DELAY)
    rospy.loginfo("Step start detected!")

    step_over = False

def wait_human_choice(step: ConM.Step):
    global HC, step_over, g_previous_elapsed

    log_event("R_S_WAIT_HC")

    rospy.loginfo("Waiting for human to act...")
    bar = IncrementalBar('Waiting human choice', max=TIMEOUT_DELAY)
    start_waiting_time = rospy.get_rostime()
    g_previous_elapsed = -1
    timeout_reached = True
    while not rospy.is_shutdown() and (rospy.get_rostime()-start_waiting_time).to_sec()<TIMEOUT_DELAY:
        elapsed = (rospy.get_rostime()-start_waiting_time).to_sec()
        bar.goto(elapsed)
        update_hmi_timeout_progress(elapsed)
        if HC!=None:
            timeout_reached = False
            break
        rospy.sleep(0.1)
    bar.goto(TIMEOUT_DELAY)
    bar.finish()

    if timeout_reached:
        g_hmi_timeout_reached_pub.publish(EmptyM()) # Rename with step started
    if HC==None:
        rospy.loginfo("Timeout reached, human not acting...")
        log_event("R_TIMEOUT")
        log_event("R_E_WAIT_HC")
        human_acting = False
    else:
        # rospy.sleep(WAIT_START_DELAY)
        rospy.loginfo("Step start detected!")
        log_event("R_E_WAIT_HC")
        if HC.is_passive():
            rospy.loginfo("Human not acting...")
            human_acting = False
        else:
            rospy.loginfo(f"Human is active")
            human_acting = True

    step_over = False
    return human_acting

g_previous_elapsed = -1
def update_hmi_timeout_progress(elapsed):
    global g_previous_elapsed
    elapsed = int(elapsed)
    if elapsed != g_previous_elapsed:
        g_hmi_timeout_value_pub.publish(elapsed)
        g_previous_elapsed = elapsed

##################
## SubFonctions ##
##################
def log_event(name):
    msg = EventLog()
    msg.name = name
    msg.timestamp = rospy.get_time()
    g_event_log_pub.publish(msg)

def human_active():
    return HC!=None and not HC.is_passive()

def ID_needed(step: ConM.Step):
    best_ra = step.human_options[0].best_robot_pair.robot_action
    for ho in step.human_options[1:]:
        if not CM.Action.are_similar( best_ra, ho.best_robot_pair.robot_action ):
            rospy.loginfo("ID Needed")
            return True
    rospy.loginfo("ID NOT Needed")
    return False

def ID_successful(result: CM.Action | None):
    return result!=None

def pick_best_RA(curr_step: ConM.Step):
    return curr_step.best_robot_pair.robot_action

def pick_best_RA_H_passive(curr_step: ConM.Step):
    for ho in curr_step.human_options:
        if not ho.human_action.is_passive():
            continue
        else:
            return ho.best_robot_pair.robot_action
    return default_robot_passive_action

def pick_best_valid_RA(step: ConM.Step, human_action: CM.Action):
    for ho in step.human_options:
        if ho.human_action==human_action:
            return ho.best_robot_pair.robot_action
    raise Exception("No best robot action defined...")

def pick_valid_passive(step: ConM.Step):
    # Find the first passive robot action
    for ho in step.human_options:
        for p in ho.action_pairs:
            if p.robot_action.is_passive():
                return p.robot_action

    raise Exception("Didn't find passive action for failed ID...")

def wait_step_end():
    rospy.loginfo("Waiting step end...")
    while not rospy.is_shutdown() and not step_over:
        rospy.sleep(0.1)
    rospy.loginfo("Current step is over.")
    log_event("R_E_WAIT_END_HA")

def get_executed_pair(step: ConM.Step, HA: CM.Action, RA: CM.Action):
    for ho in step.human_options:
        if CM.Action.are_similar(ho.human_action,HA):
            human_option = ho
            break
    pair = None
    for p in human_option.action_pairs:
        if CM.Action.are_similar(p.robot_action,RA):
            pair = p
            break
    if pair==None:
        raise WrongException("Wrong Human action identified and a conflict occured... (No corresponding parallel pair to recover).")
    return pair

def get_next_step(step: ConM.Step, HA: CM.Action, RA: CM.Action):
    if HC==None:
        return step
    else:
        executed_pair = get_executed_pair(step, HA, RA)
        first_next_pair = executed_pair.next[0]
        return first_next_pair.get_in_step()

def get_lrd_probas(nb_human_actions):
    if nb_human_actions<=1:
        raise Exception("nb_human_actions MUST BE > 1. There must be at least one HA and LRD.")

    not_eq_p = 0.0
    n = nb_human_actions+1
    if P_LET_ROBOT_DECIDE != -1:
        not_eq_p += P_LET_ROBOT_DECIDE
        n-=1
    if P_LEAVE_ROBOT_DO != -1:
        not_eq_p += P_LEAVE_ROBOT_DO
        n-=1

    p_eq = (1 - not_eq_p)/n
    p_lrde = p_eq if P_LET_ROBOT_DECIDE==-1 else P_LET_ROBOT_DECIDE
    p_lrdo = p_eq if P_LEAVE_ROBOT_DO==-1 else P_LEAVE_ROBOT_DO

    return p_lrde, p_lrdo, p_eq

def convert_rank_to_score(rank, nb):
    return -1/(nb-1) * rank + nb/(nb-1)
    return rank

def reset_human():
    global HC, g_possible_human_actions
    HC = None
    g_possible_human_actions = []

#########
## ROS ##
#########
def update_vha(valid_human_actions: list[CM.Action], start):
    msg = VHA()

    # Set Type
    msg.type = msg.START if start else msg.CONCURRENT

    # Remove passive actions
    for ha in valid_human_actions:
        if ha.is_passive():
            continue
        msg.valid_human_actions.append( ha.name + str(ha.parameters) )

    g_update_VHA_pub.publish(msg)

def compute_msg_action(a):
    msg = Action()

    msg.type = -1

    if "pick"==a.name:
        msg.type=Action.PICK_OBJ
        msg.color=a.parameters[0]
        msg.side=a.parameters[1]
    elif "place"==a.name:
        msg.type=Action.PLACE_OBJ
        msg.color=a.parameters[0]
        msg.location=a.parameters[1]
    elif "push"==a.name:
        msg.type=Action.PUSH
    elif "open_box"==a.name:
        msg.type=Action.OPEN_BOX
    elif "drop"==a.name:
        msg.type=Action.DROP
        msg.color=a.parameters[0]

    elif a.is_passive():
        msg.type=Action.PASSIVE

    if msg.type==-1:
        raise Exception("Unknown action")
        
    return msg

def start_execute_RA(RA: CM.Action):
    rospy.loginfo(f"Execute Robot Action {RA}")
    msg = compute_msg_action(RA)
    g_robot_action_pub.publish(msg)

HC = None
g_possible_human_actions = []
default_human_passive_action = None
default_robot_passive_action = None

def step_over_cb(msg):
    global step_over, HC, g_possible_human_actions
    step_over = True
    
def start_human_action_server(req):
    if req.data==-1:
        # Human is passive
        # find passive action
        pass_ha = None
        for ha in g_possible_human_actions:
            if ha.is_passive():
                pass_ha = ha
                break
        if pass_ha==None: # Pass ha not found
            pass_ha = default_human_passive_action
        HA = pass_ha

    # Regular action
    else:
        HA = g_possible_human_actions[req.data-1]

    msg_ha = compute_msg_action(HA)
    msg_ha.id = req.data
    g_human_action_pub.publish(msg_ha)

    return IntResponse()

def human_visual_signal_cb(msg: Signal):
    global HC, g_possible_human_actions, step_over

    print(f"CB human visual signal: {msg}")
    print("Signal.H_PASS= ", Signal.H_PASS)
    print("Signal.S_HA= ", Signal.S_HA)
    print("Signal.E_HA= ", Signal.E_HA)

    # Human is passive
    if msg.type == Signal.H_PASS:
        # find passive action
        pass_ha = None
        for ha in g_possible_human_actions:
            if ha.is_passive():
                pass_ha = ha
                break
        if pass_ha==None: # Pass ha not found
            # double skip, should repeat 
            pass_ha = default_human_passive_action
        HC = pass_ha
        rospy.loginfo(f"\nHuman visual : {HC}")

    # Regular action
    elif msg.type == Signal.S_HA:
        HC = g_possible_human_actions[msg.id-1]
        rospy.loginfo(f"\nHuman visual : {HC}")


##########
## MAIN ##
##########
def show_solution_exec():
    ConM.render_tree(begin_step)

def main_exec(domain_name, solution_tree, begin_step,r_p,h_p, r_ranked_leaves, h_ranked_leaves):
    global P_SUCCESS_ID_PHASE, HUMAN_TYPE, HUMAN_UPDATING, P_LET_ROBOT_DECIDE, P_LEAVE_ROBOT_DO, WAIT_START_DELAY, TIMEOUT_DELAY, ID_DELAY, ASSESS_DELAY
    global default_human_passive_action, default_robot_passive_action

    # Mock Delays
    TIMEOUT_DELAY       = 4.0
    WAIT_START_DELAY    = 0.5
    ID_DELAY            = 1.0
    ASSESS_DELAY        = 0.5
    # Mock ID phase Probabilities
    P_SUCCESS_ID_PHASE  = 1.0

    # Mock human behavior 
    #   "POLICY"  => 
    #   "RANDOM"  => Act randomly
    HUMAN_TYPE = "POLICY"
    HUMAN_UPDATING = False

    # Proba for random human(set to -1 to make it equiprobable) (must have P_LRDe + P_LRDo <= 1)
    P_LEAVE_ROBOT_DO    = -1
    P_LET_ROBOT_DECIDE  = -1

    # Init timeout delay HMI
    g_hmi_timeout_max_client(int(TIMEOUT_DELAY))

    rospy.sleep(0.5)

    # Init Seed
    seed = random.randrange(sys.maxsize)
    random.seed(seed)
    lg.debug(f"\nSeed was: {seed}")

    initDomain()

    default_human_passive_action = CM.Action.create_passive(CM.g_human_name, "PASS")
    default_robot_passive_action = CM.Action.create_passive(CM.g_robot_name, "PASS")

    if INPUT:
        rospy.loginfo("Press Enter to start...")
        input()

    try:
        id,r_rank,h_rank, r_ranked_leaves, h_ranked_leaves = execution_simulation(begin_step,r_p,h_p, r_ranked_leaves, h_ranked_leaves)
        nb_sol = len(begin_step.get_final_leaves())
        # print(r_rank,h_rank,nb_sol)
        r_score = convert_rank_to_score(r_rank,nb_sol)
        h_score = convert_rank_to_score(h_rank,nb_sol)
        return (id,r_score,h_score, seed, r_ranked_leaves, h_ranked_leaves)
    except WrongException as inst:
        lg.debug(f"Exception catched: {inst.args[0]}")
        return (-1, seed)

def find_r_rank_of_id(steps, id):
    for s in steps:
        if s.id == id:
            return s.get_f_leaf().branch_rank_r


g_update_VHA_pub = None
g_robot_action_pub = None
g_human_action_pub = None
g_hmi_timeout_value_pub = None
g_hmi_timeout_max_client = None
g_hmi_timeout_reached_pub = None
g_hmi_enable_buttons_pub = None
g_hmi_finish_pub = None
g_best_human_action = None
g_hmi_r_idle_client = None
g_r_event_log_pub = None
g_h_event_log_pub = None
g_go_idle_pose_pub = None
g_go_home_pose_pub = None
g_go_idle_pose_client = None
g_go_home_pose_client = None
g_robot_pass_pub = None
g_event_log_pub = None
if __name__ == "__main__":
    sys.setrecursionlimit(100000)

    # ROS Startup
    rospy.init_node('exec_automaton')


    g_update_VHA_pub = rospy.Publisher('/hmi_vha', VHA, queue_size=1)
    g_robot_action_pub = rospy.Publisher('/robot_action', Action, queue_size=1)
    g_human_action_pub = rospy.Publisher('/human_action', Action, queue_size=1)
    g_hmi_timeout_value_pub = rospy.Publisher('/hmi_timeout_value', Int32, queue_size=1)
    g_hmi_timeout_reached_pub = rospy.Publisher('/hmi_timeout_reached', EmptyM, queue_size=1)
    g_hmi_enable_buttons_pub = rospy.Publisher('/hmi_enable_buttons', Bool, queue_size=1)
    g_hmi_finish_pub = rospy.Publisher('/hmi_finish', EmptyM, queue_size=1)
    g_best_human_action = rospy.Publisher('/mock_best_human_action', Int32, queue_size=1)
    g_event_log_pub = rospy.Publisher('/event_log', EventLog, queue_size=10)
    g_robot_next_step_idle_pub = rospy.Publisher("/ns_idle", EmptyM, queue_size=1)
    g_robot_next_step_pub = rospy.Publisher("/ns", EmptyM, queue_size=1)
    g_robot_start_action_pub = rospy.Publisher("/s_ra", EmptyM, queue_size=1)
    g_robot_pass_pub = rospy.Publisher("/r_pass", EmptyM, queue_size=1)

    step_over_sub = rospy.Subscriber('/step_over', EmptyM, step_over_cb)
    human_visual_signal_sub = rospy.Subscriber('/human_visual_signals', Signal, human_visual_signal_cb)


    rospy.loginfo("Wait pub/sub to be initialized...")
    rospy.sleep(0.5)
    rospy.loginfo("Wait for hmi to be started...")
    rospy.wait_for_service("hmi_started")
    rospy.sleep(0.5)

    g_hmi_timeout_max_client = rospy.ServiceProxy("hmi_timeout_max", Int)
    g_hmi_r_idle_client = rospy.ServiceProxy("hmi_r_idle", SetBool)
    g_go_idle_pose_client = rospy.ServiceProxy("go_idle_pose", EmptyS)
    g_go_home_pose_client = rospy.ServiceProxy("go_home_pose", EmptyS)

    start_human_action_service = rospy.Service("start_human_action", Int, start_human_action_server)
    


    # Solution loading + characterization 
    domain_name, solution_tree, begin_step = load_solution()
    estimations = {
        "optimal": [[
            ("TimeTaskCompletion",  False),
            ("GlobalEffort",        False),
            ("TimeEndHumanDuty",    False),
            ("HumanEffort",         False),
            # ("RiskConflict",      False),
        ],[
            ("TimeTaskCompletion",  False),
            ("GlobalEffort",        False),
            ("TimeEndHumanDuty",    False),
            ("HumanEffort",         False),
            # ("RiskConflict",      False),
        ]],

        "aligned": [[
            ("TimeEndHumanDuty",    False),
            ("HumanEffort",         False),
            ("TimeTaskCompletion",  False),
            ("GlobalEffort",        False),
            # ("RiskConflict",      False),
        ],[
            ("TimeEndHumanDuty",    False),
            ("HumanEffort",         False),
            ("TimeTaskCompletion",  False),
            ("GlobalEffort",        False),
            # ("RiskConflict",      False),
        ]],

        "adversarial": [[
            ("TimeEndHumanDuty",    False),
            ("HumanEffort",         False),
            ("TimeTaskCompletion",  False),
            ("GlobalEffort",        False),
            # ("RiskConflict",      False),
        ],[
            ("TimeEndHumanDuty",    True),
            ("HumanEffort",         True),
            ("TimeTaskCompletion",  True),
            ("GlobalEffort",        True),
            # ("RiskConflict",      False),
        ]],

        "not_aligned_1": [[
            ("TimeEndHumanDuty",    False),
            ("TimeTaskCompletion",  False),
            ("GlobalEffort",        False),
            ("HumanEffort",         False),
            # ("RiskConflict",      False),
        ],[
            ("TimeTaskCompletion",  False),
            ("HumanEffort",         True),
            ("TimeEndHumanDuty",    False),
            ("GlobalEffort",        True),
            # ("RiskConflict",      False),
        ]],

        "not_aligned_2": [[
            ("GlobalEffort",        False),
            ("TimeEndHumanDuty",    False),
            ("HumanEffort",         False),
            ("TimeTaskCompletion",  False),
            # ("RiskConflict",      False),
        ],[
            ("GlobalEffort",        True),
            ("HumanEffort",         True),
            ("TimeEndHumanDuty",    False),
            ("TimeTaskCompletion",  False),
            # ("RiskConflict",      False),
        ]],

        "test": [[
            ("TimeTaskCompletion",  False),
            ("GlobalEffort",        False),
            ("TimeEndHumanDuty",    False),
            ("HumanEffort",         False),
            # ("RiskConflict",      False),
        ],[
            ("HumanEffort",         False),
            ("TimeEndHumanDuty",    False),
            ("GlobalEffort",        False),
            ("TimeTaskCompletion",  False),
            # ("RiskConflict",      False),
        ]],
    }
    r_criteria = estimations["optimal"][0]
    h_criteria = estimations["optimal"][1]
    r_ranked_leaves, h_ranked_leaves = set_choices(begin_step,r_criteria,h_criteria)

    # Execution simulation
    result = main_exec(domain_name, solution_tree, begin_step,r_criteria,h_criteria, r_ranked_leaves, h_ranked_leaves)

    # Result plot
    do_plot=False
    solutions = []
    if result[0]==-1:
        rospy.loginfo("Failed... -1")
    else:
        (id, r_score, h_score, seed, r_ranked_leaves, h_ranked_leaves) = result

        # find step with with id
        nb_sols = len(h_ranked_leaves)
        solution_step = None
        for l in begin_step.get_final_leaves():
            if l.id == id:
                solution_step = l
                break
        score_r = convert_rank_to_score(solution_step.get_f_leaf().branch_rank_r, nb_sols)
        score_h = convert_rank_to_score(solution_step.get_f_leaf().branch_rank_h, nb_sols)
        solutions.append( (score_h,score_r) )
        # print(f"solution: r#{score_r:.2f}-h#{score_h:.2f}-{nb_sols}")

        
        if do_plot:
            xdata = []
            ydata = []
            nb_sols = len(h_ranked_leaves)
            for l in h_ranked_leaves:
                xdata.append( convert_rank_to_score(l.get_f_leaf().branch_rank_h, nb_sols) )
                ydata.append( convert_rank_to_score(find_r_rank_of_id(h_ranked_leaves, l.id),nb_sols) )

            # plt.figure(figsize=(3, 3))
            plt.plot(xdata, ydata, 'b+')
            plt.plot([score_h], [score_r], 'ro')
            plt.xlabel("score human solution")
            plt.ylabel("score robot solution")
            plt.show()
    rospy.loginfo("\nfinish")

    # dill.dump(solutions, open("/home/afavier/ws/HATPEHDA/domains_and_results/solution_exec.p", "wb"))
    # print(solutions)


