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
sys.path.insert(0, "/home/afavier/exec_simulator_ws/src/progress/")
from progress.bar import IncrementalBar, StrBar
from std_msgs.msg import Int32, Bool
from std_msgs.msg import Empty as EmptyM
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptyS
from std_srvs.srv import SetBool
from sim_msgs.msg import Action, VHA
from sim_msgs.msg import EventLog
from sim_msgs.srv import Int, IntResponse, IntRequest
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

step_over = False

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
    while not exec_over(curr_step) and not rospy.is_shutdown():

        rospy.loginfo(f"Step {curr_step.id} begins.")

        if curr_step.isRInactive():
            g_text_plugin_pub.publish(String(f"Step started\nGoing IDLE\nWaiting for human to act..."))
            send_NS_update_HAs(curr_step, VHA.NS_IDLE)
            go_idle_pose_once()
            RA = select_valid_passive(curr_step)
            wait_human_start_acting(curr_step)

        else:
            go_home_pose_once()
            send_NS_update_HAs(curr_step, VHA.NS)
            wait_human_decision(curr_step)
            # MOCK_save_best_reachable_solution_for_human(curr_step)

            ## 1 & 2 & 3 ##
            if human_active(): 
                ## 1 & 2 ##
                if ID_needed(curr_step): 
                    result_id = MOCK_run_id_phase(curr_step)
                    ## 1 ##
                    if ID_successful(result_id): 
                        RA = select_best_compliant_RA(curr_step, result_id)
                    ## 2 ##
                    else: 
                        RA = select_valid_passive(curr_step)
                ## 3 ##
                else: 
                    lg.debug("ID not needed.")
                    RA = select_best_RA(curr_step)
            ## 4 ##
            else: 
                RA = select_best_RA_H_passive(curr_step)

            start_execute_RA(RA)
            passive_update_HAs(curr_step, RA)
                  
        wait_step_end()
        
        HA = MOCK_assess_human_action()

        # Check Passive Step
        if RA.is_passive() and HA.is_passive():
            # Repeat current step
            reset()
            rospy.sleep(0.1)
        else:
            curr_step = get_next_step(curr_step, HA, RA)

            # Look for degradation 
            # MOCK_save_best_reachable_solution_for_human_after_robot_choice(curr_step)
            # if MOCK_robot_has_degraded_human_best_solution():
            #     nb_of_degradation +=1
            #     if nb_of_degradation >= 1:
            #         r_ranked_leaves, h_ranked_leaves = adjust_robot_preferences(begin_step,h_pref,h_pref)

            reset()
            rospy.sleep(0.1)

    log_event("OVER")
    lg.info(f"END => {curr_step}")
    print(f"END => {curr_step}")
    g_text_plugin_pub.publish(String(f"Task Done"))
    g_hmi_finish_pub.publish(EmptyM())
    # return int(curr_step.id)
    return int(curr_step.id), curr_step.get_f_leaf().branch_rank_r, curr_step.get_f_leaf().branch_rank_h, r_ranked_leaves, h_ranked_leaves


#########################
## MOCK Human behavior ##
#########################
g_possible_human_actions = []
def send_NS_update_HAs(step: ConM.Step, type):
    global g_possible_human_actions

    sgl = Signal()
    if type==VHA.NS:
        sgl.type = Signal.NS
    elif type==VHA.NS_IDLE:
        sgl.type = Signal.NS_IDLE
    else:
        raise Exception("Invalid type to send NS signal")
    robot_visual_signal_pub.publish(sgl)

    g_possible_human_actions = [ho.human_action for ho in step.human_options]

    send_vha(g_possible_human_actions, type)

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

def passive_update_HAs(step: ConM.step, RA: CM.Action):
    global g_possible_human_actions

    if not human_active():
        # find compliant human actions and send VHA
        compliant_pairs = find_compliant_pairs_with_RA(step, RA)
        g_possible_human_actions = [p.human_action for p in compliant_pairs]
        send_vha(g_possible_human_actions, VHA.CONCURRENT)

        # find best human action in compliant actions
        ## find best pair
        best_rank_i = 0
        best_rank_v = compliant_pairs[best_rank_i].best_rank_h
        for i,p in enumerate(compliant_pairs[1:]):
            if p.best_rank_h < best_rank_v:
                best_rank_i = i
                best_rank_v = p.best_rank_h
        ## find id of corresponding best action
        best_ha = Int32()
        if compliant_pairs[best_rank_i].human_action.is_passive():
            best_ha.data = -1
        else:
            for i,ho in enumerate(step.human_options):
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


#########################################
## Preference and Solution degradation ##
#########################################
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

##########################
## MOCK Robot execution ##
##########################
def MOCK_run_id_phase(step: ConM.Step):
    """
    Simulate the identification phase.
    Wait ID_DELAY then identify human action with a P_SUCCESS_ID chance. 
    """
    log_event("R_S_ID")
    g_text_plugin_pub.publish(String("Indentifying Human action"))

    rospy.loginfo("Start ID phase...")
    rospy.sleep(ID_DELAY)
    if random.random()<P_SUCCESS_ID_PHASE:
        rospy.loginfo("ID Success")
        id_result = hidden_HC
    else:
        rospy.loginfo("ID failed...")
        id_result = None
    
    log_event("R_E_ID")
    
    return id_result

def MOCK_assess_human_action() -> CM.Action:
    global hidden_HC
    # If Human didn't choose, we try to find passive human action
    log_event("R_S_ASSESS")

    if hidden_HC==None:
        for ha in g_possible_human_actions:
            if ha.is_passive():
                hidden_HC = ha
                break
        # If didn't find passive HA
        if hidden_HC==None: 
            rospy.loginfo("Inactive step...")
            hidden_HC = default_human_passive_action
    else:
        rospy.loginfo(f"Assessed Human action: {hidden_HC}")

    rospy.sleep(ASSESS_DELAY)
    log_event("R_E_ASSESS")
    
    return hidden_HC


#######################################
## SWITCH BETWEEN IDLE AND HOME POSE ##
#######################################
R_idle = False
def go_idle_pose_once():
    global R_idle
    if not R_idle:
        g_hmi_r_idle_client.call(True)
        g_go_idle_pose_client.call()
        R_idle = True
def go_home_pose_once():
    global R_idle
    if R_idle:
        g_text_plugin_pub.publish(String(f"Going out of IDLE mode..."))
        g_hmi_r_idle_client.call(False)
        g_go_home_pose_client.call()
        R_idle = False


#######################
## Waiting Functions ##
#######################
def wait_human_start_acting(step: ConM.Step):
    rospy.loginfo("Waiting for human to act...")
    log_event("R_S_WAIT_HSA")
    while not rospy.is_shutdown():
        if hidden_HC!=None:
            break
        rospy.sleep(0.1)

    log_event("R_E_WAIT_HSA")
    rospy.loginfo("Step start detected!")

def wait_human_decision(step: ConM.Step):
    log_event("R_S_WAIT_HC")

    rospy.loginfo("Waiting for human to act...")

    bar = IncrementalBar('Waiting human choice', max=TIMEOUT_DELAY)
    str_bar = StrBar(max=TIMEOUT_DELAY, width=15)

    start_waiting_time = rospy.get_rostime()
    timeout_reached = True

    while not rospy.is_shutdown() and not step.isHInactive() and (rospy.get_rostime()-start_waiting_time).to_sec()<TIMEOUT_DELAY+ESTIMATED_R_REACTION_TIME:
        elapsed = (rospy.get_rostime()-start_waiting_time).to_sec()

        # Update progress bars
        bar.goto(elapsed)
        update_hmi_timeout_progress(elapsed)
        str_bar.goto(elapsed)
        g_text_plugin_pub.publish(String(f"Step started\nRobot waiting for human decision...\n{str_bar.get_str()}"))

        # Check termination condition
        if g_human_decision!=None:
            timeout_reached = False
            break

        # Loop rate
        rospy.sleep(0.1)

    # Finish progress bars
    bar.goto(bar.max)
    str_bar.goto(str_bar.max)
    update_hmi_timeout_progress(TIMEOUT_DELAY)
    bar.finish()
    str_bar.finish()
    
    # If Timeout Reached
    if timeout_reached:
        g_hmi_timeout_reached_pub.publish(EmptyM()) # Rename with step started
    if g_human_decision==None:
        rospy.loginfo("Timeout reached, human not acting...")
        g_text_plugin_pub.publish(String(f"Timeout reached\nHuman not acting"))
        sgl = Signal()
        sgl.type = Signal.TO
        robot_visual_signal_pub.publish(sgl)
        human_acting = False
    # Visual signal received, either PASS or Start Action
    else:
        rospy.loginfo("Step start detected!")
        if g_human_decision.type == Signal.H_PASS:
            rospy.loginfo("Human not acting...")
            human_acting = False
        else:
            rospy.loginfo(f"Human is active")
            human_acting = True

    log_event("R_E_WAIT_HC")
    return human_acting

def wait_step_end():
    rospy.loginfo("Waiting step end...")
    text_updated = False
    while not rospy.is_shutdown() and not step_over:
        if not text_updated and g_robot_action_over:
            g_text_plugin_pub.publish(String("Waiting end of human action..."))
        rospy.sleep(0.1)

    if human_active():
        rospy.sleep(ESTIMATED_R_REACTION_TIME)

    rospy.loginfo("Current step is over.")
    log_event("R_E_WAIT_STEP_END")


##################
## SubFonctions ##
##################
def log_event(name):
    msg = EventLog()
    msg.name = name
    msg.timestamp = rospy.get_time()
    g_event_log_pub.publish(msg)

def human_active():
    return hidden_HC!=None and not hidden_HC.is_passive()

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

g_previous_elapsed = -1
def update_hmi_timeout_progress(elapsed):
    global g_previous_elapsed
    elapsed = int(elapsed)
    if elapsed != g_previous_elapsed:
        g_hmi_timeout_value_pub.publish(elapsed)
        g_previous_elapsed = elapsed

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
    executed_pair = get_executed_pair(step, HA, RA)
    first_next_pair = executed_pair.next[0]
    return first_next_pair.get_in_step()

def convert_rank_to_score(rank, nb):
    return -1/(nb-1) * rank + nb/(nb-1)
    return rank

def reset():
    global hidden_HC, g_possible_human_actions, g_robot_action_over, g_human_decision, step_over, g_previous_elapsed
    hidden_HC = None
    g_possible_human_actions = []
    g_robot_action_over = False
    g_human_decision = None
    step_over = False
    g_previous_elapsed = -1

def get_first_step(begin_step: ConM.Step):
    if len(begin_step.children)!=1:
        raise Exception("begin_step should only have 1 child.")
    first_step = begin_step.children[0]
    return first_step

def get_agents_before_step(step: ConM.Step):
    return step.from_pair.end_agents

def exec_over(step):
    return step.is_final


#########################
## Select Robot Action ##
#########################
def select_best_RA(curr_step: ConM.Step):
    return curr_step.best_robot_pair.robot_action

def select_best_RA_H_passive(curr_step: ConM.Step) -> CM.Action:
    for ho in curr_step.human_options:
        if not ho.human_action.is_passive():
            continue
        else:
            return ho.best_robot_pair.robot_action
    return default_robot_passive_action

def select_best_compliant_RA(step: ConM.Step, human_action: CM.Action) -> CM.Action:
    for ho in step.human_options:
        if ho.human_action==human_action:
            return ho.best_robot_pair.robot_action
    raise Exception("No best robot action defined...")

def select_valid_passive(step: ConM.Step) -> CM.Action:
    # Find the first passive robot action
    for ho in step.human_options:
        for p in ho.action_pairs:
            if p.robot_action.is_passive():
                return p.robot_action

    raise Exception("Didn't find passive action for failed ID...")


#########
## ROS ##
#########
def send_vha(valid_human_actions: list[CM.Action], type):
    msg = VHA()
    msg.type = type

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

    if RA.is_passive():
        g_text_plugin_pub.publish(String("Robot is passive..."))
    else:
        g_text_plugin_pub.publish(String("Robot is acting"))

    msg = compute_msg_action(RA)
    g_robot_action_pub.publish(msg)

def step_over_cb(msg):
    global step_over
    step_over = True
    
def start_human_action_server(req: IntRequest):
    # Convert Int id 2 Action
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

    # Send Human action to simu controller
    msg_ha = compute_msg_action(HA)
    msg_ha.id = req.data
    g_human_action_pub.publish(msg_ha)

    return IntResponse()

g_human_decision = None
hidden_HC = None
def human_visual_signal_cb(msg: Signal):
    global hidden_HC, g_human_decision

    print(f"\nCB human visual signal: {msg}")

    rospy.sleep(ESTIMATED_R_REACTION_TIME)

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
        hidden_HC = pass_ha
        g_human_decision = msg
        rospy.loginfo(f"\nHuman visual : PASS")

    # Regular action
    elif msg.type == Signal.S_HA:
        hidden_HC = g_possible_human_actions[msg.id-1]
        g_human_decision = msg
        rospy.loginfo(f"\nHuman visual S_HA")

g_robot_action_over = False
def robot_visual_signal_cb(msg: Signal):
    global g_robot_action_over
    if msg.type == Signal.E_RA:
        g_robot_action_over = True
        rospy.loginfo("Robot action over.")


##########
## MAIN ##
##########
def show_solution_exec():
    ConM.render_tree(begin_step)

def main_exec(domain_name, solution_tree, begin_step,r_p,h_p, r_ranked_leaves, h_ranked_leaves):
    global TIMEOUT_DELAY, ESTIMATED_R_REACTION_TIME, P_SUCCESS_ID_PHASE, ID_DELAY, ASSESS_DELAY
    global default_human_passive_action, default_robot_passive_action

    rospy.loginfo("READY TO START, PRESS RETURN")
    input()

    # CONSTANTS #
    TIMEOUT_DELAY               = 10.0
    ESTIMATED_R_REACTION_TIME   = 1.0
    P_SUCCESS_ID_PHASE          = 1.0
    ID_DELAY                    = 2.0
    ASSESS_DELAY                = 0.5


    HUMAN_UPDATING = False

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

if __name__ == "__main__":
    sys.setrecursionlimit(100000)

    # ROS Startup
    rospy.init_node('exec_automaton')


    g_update_VHA_pub = rospy.Publisher('/hmi_vha', VHA, queue_size=1)
    g_robot_action_pub = rospy.Publisher('/robot_action', Action, queue_size=1)
    g_human_action_pub = rospy.Publisher('/human_action', Action, queue_size=1)
    g_hmi_timeout_value_pub = rospy.Publisher('/hmi_timeout_value', Int32, queue_size=1)
    g_hmi_timeout_reached_pub = rospy.Publisher('/hmi_timeout_reached', EmptyM, queue_size=1)
    g_hmi_finish_pub = rospy.Publisher('/hmi_finish', EmptyM, queue_size=1)
    g_best_human_action = rospy.Publisher('/mock_best_human_action', Int32, queue_size=1)
    g_event_log_pub = rospy.Publisher('/event_log', EventLog, queue_size=10)
    g_text_plugin_pub = rospy.Publisher("/text_gazebo_label", String, queue_size=1)

    step_over_sub = rospy.Subscriber('/step_over', EmptyM, step_over_cb)
    human_visual_signal_sub = rospy.Subscriber('/human_visual_signals', Signal, human_visual_signal_cb)
    robot_visual_signal_sub = rospy.Subscriber('/robot_visual_signals', Signal, robot_visual_signal_cb)
    robot_visual_signal_pub = rospy.Publisher('/robot_visual_signals', Signal, queue_size=1)


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


