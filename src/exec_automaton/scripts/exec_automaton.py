#! /usr/bin/python3
from __future__ import annotations
from typing import Any, Dict, List, Tuple
from copy import deepcopy
import random
import pickle
import dill
import sys
from enum import Enum
import logging as lg
import logging.config
import rospy
from sim_msgs.msg import Action, VHA
from std_msgs.msg import Int32, Empty

class IdResult(Enum):
    NOT_NEEDED=0
    FAILED=1

sys.path.insert(0, "/home/afavier/exec_simulator_ws/src/exec_automaton/scripts")
# sys.path.insert(0, "/home/afavier/ws/HATPEHDA/hatpehda")
import ConcurrentModule as ConM
# import importlib

from arrangement import *
# from conflict_pick import *
# from simple import *

g_update_VHA_pub = None
g_robot_action_pub = None
g_human_action_pub = None
g_step_over_sub = None

step_over = True

class WrongException(Exception):
    pass

## LOGGER ##
logging.config.fileConfig('/home/afavier/exec_simulator_ws/src/exec_automaton/scripts/log.conf')

#############
## LOADING ##
#############
begin_step = None
def load_solution():
    """
    Loads the previously produced solution.
    The domain name is retreived and returned and as well as the solution tree and the initial step.
    """
    path = "/home/afavier/exec_simulator_ws/src/exec_automaton/scripts/dom_n_sol.p"
    dom_n_sol = dill.load(open(path, "rb"))

    domain_name = dom_n_sol[0]
    solution_tree = dom_n_sol[1]
    init_step = solution_tree[0]

    return domain_name, solution_tree, init_step


###############
## EXECUTION ##
###############
def execution_simulation(begin_step: ConM.Step):
    """
    Main algorithm 
    """

    curr_step = get_first_step(begin_step)
    while not rospy.is_shutdown() and not exec_over(curr_step):
        human_acting = wait_step_start(curr_step)

        result_id = IdResult.NOT_NEEDED
        if not human_acting:
            ## 4 & 5 ##
            RA = pick_any_RA(curr_step)
        else:
            if is_ID_needed(curr_step):
                result_id = MOCK_run_id_phase(curr_step)
                if ID_successful(result_id):
                    ## 2 ##
                    RA = pick_any_valid_RA(curr_step, result_id)
                else:
                    ## 1 ##
                    RA = curr_step.SRA
            else:
                ## 3 ##
                rospy.loginfo("ID not needed.")
                result_id = IdResult.NOT_NEEDED
                RA = curr_step.SRA

        execute_RA(RA)

        wait_step_end()


        # MOCK_H_LRDe(curr_step, RA)
        HA = MOCK_assess_human_action(result_id)

        curr_step = get_next_step(curr_step, HA, RA)
    rospy.loginfo(f"END => {curr_step}")
    return int(curr_step.id)


#########################
## MOCK Human behavior ##
#########################
HC = None
g_p_lrde = None
g_p_lrdo = None
def MOCK_H_action_choice(step: ConM.Step):
    """
    Simulate the human choice of action.
    Common actions are equiprobable, and LRD has a defined probability P_PICK_LRD.
    LRD can be made equiprobable with common actions if P_PICK_LRD is set to -1.
    """
    global HC, g_p_lrde, g_p_lrdo
    g_p_lrde = None

    possible_human_actions = [ho.human_action for ho in step.human_options]

    if len(possible_human_actions)==1:
        HC=possible_human_actions[0]
    else:
        # Compute weights - probas
        p_lrde, p_lrdo, p_equi = get_lrd_probas(len(possible_human_actions))
        weights = [p_equi for i in range(len(possible_human_actions)-1)] + [p_lrde+p_lrdo]
        g_p_lrde = p_lrde
        g_p_lrdo = p_lrdo

        # Make random choice
        HC = random.choices(possible_human_actions, weights=weights)[0]

def MOCK_H_LRDe(step: ConM.Step, RA: CM.Action):
    """
    If the human performed a LRD action it can be a Let_Robot_Decide.
    In such case, the human actually performs another action during the step (HC must be updated)
    """
    global HC
    if HC.name=="LRD":
        p_Le_L = g_p_lrde/(g_p_lrde + g_p_lrdo)
        if random.random()<p_Le_L:
            rospy.loginfo("Human only let robot decide")
            valid_pairs = []
            for p in step.get_pairs():
                if p.human_action.name!="LRD" and CM.Action.are_similar(p.robot_action, RA):
                    valid_pairs.append(p)
            if len(valid_pairs)>0:
                HC = random.choice(valid_pairs).human_action


##########################
## MOCK Robot execution ##
##########################
def MOCK_run_id_phase(step: ConM.Step):
    """
    Simulate the identification phase.
    LRD are always identified.
    For other human actions, the ID phase has a P_SUCCESS_ID_PHASE chance to succeed.
    If successful, there is a P_WRONG_ID chance that the ID the actually wrong.
    """
    rospy.loginfo("Start ID phase...")
    if HC.name=="LRD":
        id_result = HC
    else:
        if random.random()<P_SUCCESS_ID_PHASE:
            possible_other_human_actions = [ho.human_action for ho in step.human_options if not ho.human_action.name in ["LRD", HC.name]]
            if len(possible_other_human_actions)>0 and random.random()<P_WRONG_ID:
                id_result = random.choice(possible_other_human_actions)
            else:
                id_result = HC
        else:
            rospy.loginfo("ID failed...")
            id_result = IdResult.FAILED

    if id_result!=IdResult.FAILED:
        rospy.loginfo(f"ID successful: {id_result}")
    
    return id_result

def MOCK_assess_human_action(result_id):
    rospy.loginfo(f"Assessed Human action: {HC}")
    if result_id!=IdResult.NOT_NEEDED and result_id.name!="LRD" and not CM.Action.are_similar(result_id, HC):
        rospy.loginfo("Robot was wrong about Human action...")
    return HC


##################
## SubFonctions ##
##################
def get_first_step(begin_step: ConM.Step):
    if len(begin_step.children)!=1:
        raise Exception("begin_step should only have 1 child.")
    first_step = begin_step.children[0]
    return first_step

def get_agents_before_step(step: ConM.Step):
    return step.from_pair.end_agents

def exec_over(step):
    if step.SRA.name=="IDLE":
        pairs = step.get_pairs()
        if len(pairs)==1\
            and pairs[0].human_action.name=="IDLE"\
            and pairs[0].robot_action.name=="IDLE":
                return True
    return False

def wait_step_start(step: ConM.Step):
    global HC, step_over
    # i.e. wait for human to start acting or timeout reached
    rospy.loginfo("Current step:\n"+CM.str_agents(get_agents_before_step(step)))
    rospy.loginfo(step.str())

    possible_human_actions = [ho.human_action for ho in step.human_options]
    update_vha(possible_human_actions)

    rospy.loginfo("Waiting for human to act...")
    try:
        human_choice = rospy.wait_for_message("/human_choice", Int32, timeout=TIMEOUT_DELAY)
    except rospy.exceptions.ROSException:
        rospy.loginfo("Time out reached")
        human_choice = None
        
    if human_choice==None:
        rospy.loginfo("human not acting...")
        human_acting = False
    elif possible_human_actions[human_choice.data].name=="LRD":
        rospy.loginfo("human not acting...")
        human_acting = False
        HC = possible_human_actions[human_choice.data]
    else:
        human_acting = True
        rospy.loginfo(f"human performing: {possible_human_actions[human_choice.data]}")
        HC = possible_human_actions[human_choice.data]
        msg = compute_msg_action(HC)
        g_human_action_pub.publish(msg)

    step_over = False
    return human_acting

def is_ID_needed(step: ConM.Step):
    # ID needed if SRA is WAIT and R has other options
    raw_r_actions = [p.robot_action for p in step.get_pairs()]
    #remove double
    r_actions = raw_r_actions[:]
    i = 0
    while i<len(r_actions):
        j = i+1
        while j<len(r_actions):
            if CM.Action.are_similar(r_actions[i], r_actions[j]):
                r_actions.pop(j)
            j+=1
        i+=1

    return step.SRA.name=="WAIT" and len(r_actions)>1

def ID_successful(result: CM.Action | None):
    return result!=IdResult.FAILED and result!=IdResult.NOT_NEEDED

def pick_any_RA(step: ConM.Step):
    pairs = step.get_pairs()
    robot_actions = [p.robot_action for p in pairs]

    # i=0
    # while i<len(robot_actions):
    #     ra1 = robot_actions[i]
    #     j=i+1
    #     while j<len(robot_actions):
    #         ra2 = robot_actions[j]
    #         if CM.Action.are_similar(ra1,ra2):
    #             robot_actions.pop(j)
    #         else:
    #             j+=1
    #     i+=1

    # pick any or SRA?, for now SRA
    if step.SRA.name=="SKIP":
        RA = robot_actions[0]
    else:
        RA = step.SRA

    # compute VHA
    valid_human_actions = []
    for ho in step.human_options:
        for ho_ra in ho.robot_actions:
            if CM.Action.are_similar(RA, ho_ra):
                valid_human_actions.append(ho.human_action)
    update_vha(valid_human_actions)

    return RA

def pick_any_valid_RA(step: ConM.Step, human_action: CM.Action):
    # Pick robot action according to identified human action
    # And apply this action

    for ho in step.human_options:
        if ho.human_action==human_action:
            human_option = ho
            break
    
    robot_actions = human_option.robot_actions
    robot_action = robot_actions[0]
    if robot_action.name=="WAIT" and len(robot_actions)>1:
        robot_action = robot_actions[1]

    return robot_action

def step_over_cb(msg):
    global step_over
    step_over = True

def wait_step_end():
    rospy.loginfo("Waiting step end...")
    #TODO, should liste to human choice, to publish the human action if needed ...
    while not step_over and not rospy.is_shutdown():
        rospy.sleep(0.1)
    rospy.loginfo("Current step is over.")

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


#########
## ROS ##
#########
def update_vha(valid_human_actions: list[CM.Action]):
    msg = VHA()
    msg.valid_human_actions = []
    for ha in valid_human_actions:
        msg.valid_human_actions.append( ha.name + str(ha.parameters) )
    rospy.loginfo(msg)
    g_update_VHA_pub.publish(msg)

def compute_msg_action(a):
    msg = Action()

    if"pick_b"==a.name:
        msg.type=Action.PICK_B
        msg.obj = "cube_b"
    elif "pick_r"==a.name:
        msg.type=Action.PICK_R
        msg.obj = "cube_r"
    elif "pick_g"==a.name:
        msg.type=Action.PICK_G
        msg.obj = "cube_g"
    elif "place_1"==a.name:
        msg.type=Action.PLACE_1
        msg.location = "loc_1"
    elif "place_2"==a.name:
        msg.type=Action.PLACE_2
        msg.location = "loc_2"
    elif "place_3"==a.name:
        msg.type=Action.PLACE_3
        msg.location = "loc_3"
    elif "place_4"==a.name:
        msg.type=Action.PLACE_4
        msg.location = "loc_4"
    elif "drink"==a.name:
        msg.type=Action.DRINK
    elif "pushing"==a.name:
        msg.type=Action.PUSHING
    elif "WAIT"==a.name:
        msg.type=Action.WAIT
    elif "IDLE"==a.name:
        msg.type=Action.IDLE
    else:
        raise Exception("Unknown action")
        
    return msg

def execute_RA(RA: CM.Action):
    rospy.loginfo(f"Execute Robot Action {RA}")
    msg = compute_msg_action(RA)
    g_robot_action_pub.publish(msg)


##########
## MAIN ##
##########
def show_solution_exec():
    ConM.render_tree(begin_step)

def main_exec(domain_name, solution_tree, begin_step):
    global P_SUCCESS_ID_PHASE, P_WRONG_ID, P_LET_ROBOT_DECIDE, P_LEAVE_ROBOT_DO, TIMEOUT_DELAY

    TIMEOUT_DELAY       = 500.0
    # Mock ID phase
    P_SUCCESS_ID_PHASE  = 0.6
    P_WRONG_ID          = 0.0
    # Mock human behavior (set to -1 to make it equiprobable) (must have P_LRDe + P_LRDo <= 1)
    P_LEAVE_ROBOT_DO    = 0.4
    P_LET_ROBOT_DECIDE  = 0.1
    

    # Init Seed
    seed = random.randrange(sys.maxsize)
    random.seed(seed)
    rospy.loginfo(f"Seed was: {seed}")

    initDomain()

    try:
        result = execution_simulation(begin_step)
    except WrongException as inst:
        rospy.loginfo(f"Exception catched: {inst.args[0]}")
        result=-1

    return (result, seed)

if __name__ == "__main__":
    rospy.init_node('exec_automaton')
    g_update_VHA_pub = rospy.Publisher('/hmi_vha', VHA, queue_size=1)
    g_robot_action_pub = rospy.Publisher('/robot_action', Action, queue_size=1)
    g_human_action_pub = rospy.Publisher('/human_action', Action, queue_size=1)
    g_step_over_sub = rospy.Subscriber('/step_over', Empty, step_over_cb)

    rospy.loginfo("Wait pub/sub to be initialized...")
    rospy.sleep(1)

    domain_name, solution_tree, begin_step = load_solution()
    main_exec(domain_name, solution_tree, begin_step)