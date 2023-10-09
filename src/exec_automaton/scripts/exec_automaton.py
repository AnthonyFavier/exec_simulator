#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Dict, List, Tuple
from copy import deepcopy
import random
import dill
import sys
import rospy
import logging as lg
import logging.config
import time
from enum import Enum
import matplotlib.pyplot as plt
sys.path.insert(0, "/home/afavier/new_exec_sim_ws/src/progress/")
from progress.bar import IncrementalBar, StrBar, IncrementalBarStr
from std_msgs.msg import Int32, Bool
from std_msgs.msg import Empty as EmptyM
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptyS
from std_srvs.srv import SetBool
from sim_msgs.msg import Action, VHA
from sim_msgs.msg import EventLog
from sim_msgs.srv import Int, IntResponse, IntRequest
from sim_msgs.msg import Signal
from sim_msgs.msg import HeadCmd

class IdResult(Enum):
    NOT_NEEDED=0
    FAILED=1

# stack_empiler | stack_empiler_1 | stack_empiler_2 | stack_box
DOMAIN_NAME = "stack_empiler_2"

DEBUG = False
INPUT = True
########
# DEBUG = True
# INPUT = True

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)
import ConcurrentModule as ConM
import CommonModule as CM

step_over = False

class WrongException(Exception):
    pass

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')


#############
## LOADING ##
#############
def load(filename):
    global g_domain_name
    """
    Loads the previously produced solution.
    The domain name is retreived and returned and as well as the solution tree and the initial step.
    """

    print(f"Loading solution '{filename}' ... ", end="", flush=True)
    s_t = time.time()

    g_domain_name, init_step = dill.load(open(CM.path + filename, "rb"))


    print("Loaded! - %.2fs" %(time.time()-s_t))

    return init_step

def load_solution(exec_regime, with_choices=False):
    if with_choices:
        if exec_regime=="tt":
            file_name = "dom_n_sol_tt_with_choices.p"
        else:
            file_name = "dom_n_sol_with_choices.p"
    else:
        if exec_regime=="tt":
            file_name = "dom_n_sol_tt.p"
        else:
            file_name = "dom_n_sol.p"

    return load(file_name)

def set_choices(init_step,r_criteria,h_criteria):

    print('Start policy update...')

    final_leaves = init_step.get_final_leaves()

    r_ranked_leaves = ConM.sorting_branches(final_leaves, r_criteria, is_robot=True) #type: List[ConM.Step]
    ConM.update_robot_choices(init_step)

    h_ranked_leaves = ConM.sorting_branches(final_leaves, h_criteria, is_robot=False) #type: List[ConM.Step]
    ConM.update_human_choices(init_step)

    print('Policy update done')
    return r_ranked_leaves, h_ranked_leaves

def set_r_choices(init_step,r_criteria):
    print('Start policy update...  ', end="", flush=True)
    s_t = time.time()

    final_leaves = init_step.get_final_leaves()

    r_ranked_leaves = ConM.sorting_branches(final_leaves, r_criteria, is_robot=True) #type: List[ConM.Step]
    ConM.update_robot_choices(init_step)

    print('Done! - %.2fs' %(time.time()-s_t), )
    return r_ranked_leaves

###############
## EXECUTION ##
###############
def execution_HF(begin_step: ConM.Step):
    """
    Main algorithm 
    """
    curr_step = get_first_step(begin_step)
    while not exec_over(curr_step) and not rospy.is_shutdown():

        rospy.loginfo(f"Step {curr_step.id} begins.")
        

        if curr_step.isRInactive():
            prompt("HF_idle_step_started")
            send_NS_update_HAs(curr_step, VHA.NS_IDLE)
            look_at_human()
            go_idle_pose_once()
            RA = select_valid_passive(curr_step)
            wait_human_start_acting(curr_step)

        else:
            go_home_pose_once()
            look_at_human()

            if check_if_human_is_done(curr_step):
                set_permanent_prompt_line("h_done")
                rospy.loginfo("You are done.")
            elif check_if_human_can_leave(curr_step):
                set_permanent_prompt_line("h_can_leave")
                rospy.loginfo("You can leave.")
            else:
                reset_permanent_prompt_line()

            send_NS_update_HAs(curr_step, VHA.NS)
            wait_human_decision(curr_step)

            ## 1 & 2 & 3 ##
            if human_active(): 
                reset_permanent_prompt_line()
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
            time.sleep(0.1)
        else:
            curr_step = get_next_step(curr_step, HA, RA)

            reset()
            time.sleep(0.1)

    log_event("OVER")
    reset_permanent_prompt_line()
    prompt("task_done")
    reset_head()
    go_idle_pose_once()
    lg.info(f"END => {curr_step}")
    print(f"END => {curr_step}")
    g_hmi_finish_pub.publish(EmptyM())
    return int(curr_step.id), curr_step.get_f_leaf().branch_rank_r

def execution_RF(begin_step: ConM.Step):
    """
    Main algorithm 
    """
    curr_step = get_first_step(begin_step)
    while not exec_over(curr_step) and not rospy.is_shutdown():

        rospy.loginfo(f"Step {curr_step.id} begins.")
        
        look_at_human()

        if curr_step.isRInactive():
            prompt("RF_idle_step_started")
            send_NS_update_HAs(curr_step, VHA.NS_IDLE)
            go_idle_pose_once()
            RA = select_valid_passive(curr_step)
            wait_human_start_acting(curr_step)

        else:
            go_home_pose_once()
            
            RA = select_best_RA(curr_step)
            start_execute_RA(RA)

            send_NS(VHA.NS)
            passive_update_HAs(curr_step, RA)

            if RA.is_passive():
                start_waiting_time = time.time()
                timeout_reached = True
                while not rospy.is_shutdown() and time.time()-start_waiting_time<TIMEOUT_DELAY+ESTIMATED_R_REACTION_TIME:
                    if g_human_decision!=None:
                        timeout_reached = False
                        break
                time.sleep(0.1)
                if timeout_reached:
                    RA = select_best_active_RA(curr_step)
                    start_execute_RA(RA)
                    passive_update_HAs(curr_step, RA)
                  
        wait_step_end()
        
        HA = MOCK_assess_human_action()

        # Check Passive Step
        if RA.is_passive() and HA.is_passive():
            # Repeat current step
            reset()
            time.sleep(0.1)
        else:
            curr_step = get_next_step(curr_step, HA, RA)

            reset()
            time.sleep(0.1)

    log_event("OVER")
    prompt("task_done")
    reset_head()
    go_idle_pose_once()
    lg.info(f"END => {curr_step}")
    print(f"END => {curr_step}")
    g_hmi_finish_pub.publish(EmptyM())
    return int(curr_step.id), curr_step.get_f_leaf().branch_rank_r

def execution_TT(begin_step: ConM.Step):
    global g_possible_human_actions
    """
    Main algorithm 
    """
    curr_step = get_first_step(begin_step)
    while not exec_over(curr_step) and not rospy.is_shutdown():

        rospy.loginfo(f"Step {curr_step.id} begins.")
        
        look_at_human()

        send_NS(VHA.NS)

        # identify acting agent
        p = curr_step.get_pairs()[0]

        # ROBOT TURN
        if p.human_action.is_wait_turn():
            HA = p.human_action
            RA = select_best_RA(curr_step)
            start_execute_RA(RA)

            if not RA.is_passive():
                wait_step_end()

        # HUMAN TURN
        elif p.robot_action.is_wait_turn():
            RA = p.robot_action
            g_possible_human_actions = [ho.human_action for ho in curr_step.human_options]
            send_vha(g_possible_human_actions, VHA.NS)
            wait_human_decision(curr_step)

            HA = MOCK_assess_human_action()
            if not HA.is_passive():
                wait_step_end()

        else:
            raise Exception("Unable to identify acting agent")
        
        
        
        # Check Passive Step
        if RA.is_passive() and HA.is_passive() and curr_step.from_pair.is_passive() and not curr_step.parent.id==0:
            # Repeat previous step
            curr_step = curr_step.parent
            reset()
            time.sleep(0.1)
        else:
            curr_step = get_next_step(curr_step, HA, RA)

            reset()
            time.sleep(0.1)

    log_event("OVER")
    prompt("task_done")
    reset_head()
    go_idle_pose_once()
    lg.info(f"END => {curr_step}")
    print(f"END => {curr_step}")
    g_hmi_finish_pub.publish(EmptyM())
    return int(curr_step.id), curr_step.get_f_leaf().branch_rank_r


#########################
## MOCK Human behavior ##
#########################
g_possible_human_actions = []
def send_NS_update_HAs(step: ConM.Step, type):
    global g_possible_human_actions

    
    send_NS(type)

    g_possible_human_actions = [ho.human_action for ho in step.human_options]
    send_vha(g_possible_human_actions, type)

    # Find best human action id, sent to hmi mock
    # best_ha = Int32()
    # if step.best_human_pair.human_action.is_passive():
    #     best_ha.data = -1
    # else:
    #     for i,ho in enumerate(step.human_options):
    #         if CM.Action.are_similar(ho.human_action, step.best_human_pair.human_action):
    #             best_ha.data = i+1
    #             break
    # g_best_human_action.publish(best_ha)

def send_NS(type):
    sgl = Signal()
    if type==VHA.NS:
        sgl.type = Signal.NS
    elif type==VHA.NS_IDLE:
        sgl.type = Signal.NS_IDLE
    else:
        raise Exception("Invalid type to send NS signal")
    robot_visual_signal_pub.publish(sgl)

def passive_update_HAs(step: ConM.step, RA: CM.Action):
    global g_possible_human_actions

    if not human_active():
        # find compliant human actions and send VHA
        compliant_pairs = find_compliant_pairs_with_RA(step, RA)
        g_possible_human_actions = [p.human_action for p in compliant_pairs]
        send_vha(g_possible_human_actions, VHA.CONCURRENT)

        # find best human action in compliant actions
        ## find best pair
        # best_rank_i = 0
        # best_rank_v = compliant_pairs[best_rank_i].best_rank_h
        # for i,p in enumerate(compliant_pairs[1:]):
        #     if p.best_rank_h < best_rank_v:
        #         best_rank_i = i
        #         best_rank_v = p.best_rank_h
        # ## find id of corresponding best action
        # best_ha = Int32()
        # if compliant_pairs[best_rank_i].human_action.is_passive():
        #     best_ha.data = -1
        # else:
        #     for i,ho in enumerate(step.human_options):
        #         if CM.Action.are_similar( ho.human_action, compliant_pairs[best_rank_i].human_action ):
        #             best_ha.data = i+1
        #             break
        # g_best_human_action.publish(best_ha)

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

##########################
## MOCK Robot execution ##
##########################
def MOCK_run_id_phase(step: ConM.Step):
    """
    Simulate the identification phase.
    Wait ID_DELAY then identify human action with a P_SUCCESS_ID chance. 
    """
    log_event("R_S_ID")
    prompt("ID_started")

    rospy.loginfo("Start ID phase...")
    time.sleep(ID_DELAY)
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

    time.sleep(ASSESS_DELAY)
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
        prompt("out_of_idle")
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
        time.sleep(0.1)

    log_event("R_E_WAIT_HSA")
    rospy.loginfo("Step start detected!")

def wait_human_decision(step: ConM.Step):
    log_event("R_S_WAIT_HC")

    rospy.loginfo("Waiting for human to act...")

    bar = IncrementalBar('Waiting human choice', max=TIMEOUT_DELAY)
    # str_bar = StrBar(max=TIMEOUT_DELAY, width=15)
    str_bar = IncrementalBarStr(max=TIMEOUT_DELAY, width=INCREMENTAL_BAR_STR_WIDTH)

    start_waiting_time = time.time()

    timeout_reached = True
    if step.isHInactive():
        timeout_reached = False 
    
    while not rospy.is_shutdown() and not step.isHInactive() and time.time()-start_waiting_time<TIMEOUT_DELAY+ESTIMATED_R_REACTION_TIME:
        elapsed = time.time()-start_waiting_time

        # Update progress bars
        bar.goto(elapsed)
        update_hmi_timeout_progress(elapsed)
        str_bar.goto(elapsed)
        prompt("wait_human_decision", f"\n{str_bar.get_str()}")

        # Check termination condition
        if g_human_decision!=None:
            timeout_reached = False
            break

        # Loop rate
        time.sleep(0.1)

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
        prompt("wait_h_decision_timeout")
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
    global g_robot_acting
    rospy.loginfo("Waiting step end...")
    text_updated = False
    while not rospy.is_shutdown() and not step_over:
        if not text_updated and g_robot_action_over:
            prompt("wait_end_ha")
        time.sleep(0.1)

    if human_active():
        # Because step_over isn't sent as a human visual signal....
        time.sleep(ESTIMATED_R_REACTION_TIME)

    g_robot_acting = False
    rospy.loginfo("Current step is over.")
    log_event("R_E_WAIT_STEP_END")


##################
## SubFonctions ##
##################
def log_event(name):
    msg = EventLog()
    msg.name = name
    msg.timestamp = time.time()
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
    return step.is_final()

def look_at_human():
    msg = HeadCmd()
    msg.type = HeadCmd.LOOK_AT_HUMAN
    g_head_cmd_pub.publish(msg)

def reset_head():
    msg = HeadCmd()
    msg.type = HeadCmd.RESET
    g_head_cmd_pub.publish(msg)

def check_if_human_can_leave(step):
    # human can leave now if there is from the current step a sequence of pairs of actions leading to a final leaf
    # where the human is always passive
    
    for p in step.get_pairs():
        if p.human_action.is_passive():
            if p.is_final() or (p.next!=[] and check_if_human_can_leave(p.next[0].get_in_step())):
                return True
    return False

def check_if_human_is_done(step):
    # human is done if there is from the current step a sequence of pairs of actions leading to a final leaf
    # where the human is always IDLE

    for p in step.get_pairs():
        if p.human_action.is_idle():
            if p.is_final() or (p.next!=[] and check_if_human_is_done(p.next[0].get_in_step())):
                return True
    return False

#########################
## Select Robot Action ##
#########################
def select_best_RA(curr_step: ConM.Step) -> CM.Action:
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

def select_best_active_RA(step: ConM.Step) -> CM.Action:
    best_rank_r = None
    best_ra = None
    for p in step.get_pairs():
        if not p.robot_action.is_passive() and (best_rank_r==None or p.best_rank_r < best_rank_r):
            best_rank_r = p.best_rank_r
            best_ra = p.robot_action
    return best_ra


############################
## Compute Action Message ##
############################
def compute_msg_action_stack_empiler_1(a):
    return compute_msg_action_stack_empiler(a)

def compute_msg_action_stack_empiler_2(a):
    return compute_msg_action_stack_empiler(a)

def compute_msg_action_stack_box(a):
    msg = Action()
    msg.type = -1

    if "pick"==a.name:
        msg.type=Action.PICK_OBJ_NAME
        msg.obj_name=a.parameters[0]
    elif "place"==a.name:
        msg.type=Action.PLACE_OBJ_NAME
        msg.location=a.parameters[0]
        msg.obj_name=a.parameters[1]
    elif "push"==a.name:
        msg.type=Action.PUSH
    elif "open_box"==a.name:
        msg.type=Action.OPEN_BOX
    elif "drop"==a.name:
        msg.type=Action.DROP
        msg.obj_name=a.parameters[0]

    elif a.is_passive():
        msg.type=Action.PASSIVE

    if msg.type==-1:
        raise Exception("Unknown action")
    
    return msg

def compute_msg_action_stack_empiler(a):
    msg = Action()
    msg.type = -1

    if "pick"==a.name:
        msg.type=Action.PICK_OBJ_NAME
        msg.obj_name=a.parameters[0]
    elif "place"==a.name:
        msg.type=Action.PLACE_OBJ_NAME
        msg.location=a.parameters[0]
        msg.obj_name=a.parameters[1]
    elif "push"==a.name:
        msg.type=Action.PUSH
    elif "open_box"==a.name:
        msg.type=Action.OPEN_BOX
    elif "drop"==a.name:
        msg.type=Action.DROP
        msg.obj_name=a.parameters[0]

    elif a.is_passive():
        msg.type=Action.PASSIVE

    if msg.type==-1:
        raise Exception("Unknown action")
    
    return msg

def compute_msg_action_classic(a):
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

def compute_msg_action(a):
    if DOMAIN_NAME=="stack_empiler":
        msg = compute_msg_action_stack_empiler(a)
    elif DOMAIN_NAME=="stack_empiler_1":
        msg = compute_msg_action_stack_empiler_1(a)
    elif DOMAIN_NAME=="stack_empiler_2":
        msg = compute_msg_action_stack_empiler_2(a)
    elif DOMAIN_NAME=="classic":
        msg = compute_msg_action_classic(a)
    elif DOMAIN_NAME=="stack_box":
        msg = compute_msg_action_stack_box(a)
    else:
        raise Exception("Domain_name unknown...")
        
    return msg


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

g_robot_acting = False
def start_execute_RA(RA: CM.Action):
    global g_robot_acting
    g_robot_acting = True
    rospy.loginfo(f"Execute Robot Action {RA}")

    if RA.is_passive():
        prompt("robot_is_passive")
    else:
        prompt("robot_is_acting")

    msg = compute_msg_action(RA)
    g_robot_action_pub.publish(msg)

def step_over_cb(msg):
    global step_over
    step_over = True
    
def start_human_action_server(req: IntRequest):
    if not g_robot_acting:
        msg = HeadCmd()
        msg.type = HeadCmd.FOLLOW_H_HAND
        g_head_cmd_pub.publish(msg)

    # print("g_possible_human_actions")
    # print(g_possible_human_actions)

    # print("req")
    # print(req)
    # input()
    
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

    time.sleep(ESTIMATED_R_REACTION_TIME)

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

############
## PROMPT ##
############

LANG = "FR" # ENG | FR

g_prompt_messages = {
    "start_simu_delay": {
        "ENG":  "Starting in:",
        "FR":   "Début dans :",
        },
    "HF_idle_step_started": {
        "ENG": "You must act, I will wait.",
        "FR":  "Vous devez agir, je vais patienter.",
        },
    "RF_idle_step_started": {
        "ENG": "You must act, I will wait.",
        "FR":  "Vous devez agir, je vais patienter.",
        },
    "task_done": {
        "ENG": "The task done.",
        "FR":  "Tâche terminée.",
        },
    "ID_started": {
        "ENG": "*Identifying Action*",
        "FR":  "*Identification de l'action*",
        },
    "out_of_idle": {
        "ENG": "I'm getting ready..",
        "FR":  "Je me prepare..",
        },
    "wait_human_decision": {
        "ENG": "Act if you will.",
        "FR":  "Agissez si vous le souhaitez.",
        },
    "wait_h_decision_timeout": {
        "ENG": "Then, I will start.",
        "FR":  "Je vais commencer alors.",
        },
    "wait_end_ha": {
        "ENG": "I'm waiting for you to be done.",
        "FR":  "J'attends que vous ayez terminé.",
        },
    "robot_is_passive": {
        "ENG": "I will be passive this step.",
        "FR":  "Je serai passif.",
        },
    "robot_is_acting": {
        "ENG": "I'm performing an action...",
        "FR":  "J'agis...",
        },
    "reset_world":{
        "ENG": "Resetting the world...",
        "FR":  "Reinitialisation...",
        },
    "h_done":{
        "ENG": "You are done.",
        "FR":  "Vous avez terminé.",
        },
    "h_can_leave":{
        "ENG": "I can finish alone.",
        "FR":  "Je suis capable de terminer seul.",
        },
}

g_permanent_prompt = ""
g_start_extra = "  "
def set_permanent_prompt_line(msg_id):
    global g_permanent_prompt
    g_permanent_prompt = g_start_extra + g_prompt_messages[msg_id][LANG] + "\n"
def reset_permanent_prompt_line():
    global g_permanent_prompt
    g_permanent_prompt = ""
def prompt(msg_id: str, extra=""):
    g_prompt_pub.publish(String( g_permanent_prompt + g_start_extra + g_prompt_messages[msg_id][LANG] + extra ))

##########
## MAIN ##
##########
def find_r_rank_of_id(steps, id):
    for s in steps:
        if s.id == id:
            return s.get_f_leaf().branch_rank_r

def main_exec():
    global TIMEOUT_DELAY, ESTIMATED_R_REACTION_TIME, P_SUCCESS_ID_PHASE, ID_DELAY, ASSESS_DELAY
    global default_human_passive_action, default_robot_passive_action
    global INCREMENTAL_BAR_STR_WIDTH
    global g_domain_name

    # CONSTANTS #
    #   Delays 
    TIMEOUT_DELAY               = 4.0
    ESTIMATED_R_REACTION_TIME   = 0.3
    ID_DELAY                    = 1.0
    ASSESS_DELAY                = 0.2
    #   Proba
    P_SUCCESS_ID_PHASE          = 1.0


    HUMAN_UPDATING = False

    START_SIMU_DELAY            = 5.0
    INCREMENTAL_BAR_STR_WIDTH = 26

    time.sleep(0.5)

    # Init Seed
    seed = random.randrange(sys.maxsize)
    random.seed(seed)
    lg.debug(f"\nSeed was: {seed}")

    default_human_passive_action = CM.Action.create_passive("H", "PASS")
    default_robot_passive_action = CM.Action.create_passive("R", "PASS")

    ## LOADING ##
    sol_tee =       load("stack_empiler_2_tee.p")
    sol_hmw =       load("stack_empiler_2_hmw.p")
    sol_tt_tee =    load("stack_empiler_2_tt_tee.p")
    sol_tt_hmw =    load("stack_empiler_2_tt_hmw.p")
    if g_domain_name!=DOMAIN_NAME:
        raise Exception("Missmatching domain names CONSTANT and loaded")
    robots = {
        "hf1" : ("hf", sol_tee),
        "rf2" : ("rf", sol_tee),

        "hf3" : ("hf", sol_hmw),
        "rf4" : ("rf", sol_hmw),
    
        "hf5" : ("hf", sol_hmw),
        "rf6" : ("rf", sol_hmw),

        "hf7" : ("hf", sol_tt_hmw),
        "tt8" : ("tt", sol_tt_hmw),

        "hf9" : ("hf", sol_tt_tee),
        "tt10": ("tt", sol_tt_tee),
    }
    robot_name = input("Which robot? ")
    if robot_name=="":
        robot_name = "hf1"

    rospy.loginfo("Wait for hmi to be started...")
    rospy.wait_for_service("hmi_started")
    g_hmi_timeout_max_client(int(TIMEOUT_DELAY))

    continuer = True
    while continuer:
        
        exec_regime, begin_step = robots[robot_name]

        if INPUT:
            rospy.loginfo("READY TO START, Press Enter to start...")
            input()
            
            bar = IncrementalBar(max = START_SIMU_DELAY)
            str_bar = IncrementalBarStr(max = START_SIMU_DELAY, width=INCREMENTAL_BAR_STR_WIDTH)

            start_time = time.time()
            while not rospy.is_shutdown() and time.time()-start_time<START_SIMU_DELAY:
                elapsed = time.time() - start_time

                bar.goto(elapsed)
                str_bar.goto(elapsed)
                prompt("start_simu_delay", f"\n{str_bar.get_str()}")

                time.sleep(0.05)

        try:
            if exec_regime == "hf":
                id,r_rank = execution_HF(begin_step)
            elif exec_regime == "rf":
                id,r_rank = execution_RF(begin_step)
            elif exec_regime == "tt":
                id,r_rank = execution_TT(begin_step)
            else:
                raise Exception("unknown exec_regime.")
            nb_sol = len(begin_step.get_final_leaves())
            r_score = convert_rank_to_score(r_rank,nb_sol)
            print(f"END: id={id}, r_score={r_score}")
            in_choice = input("repeat? (y,robot_name,n)")
            if in_choice=="y":
                prompt("reset_world")
                g_reset_world_client()
                continue
            elif in_choice=="n":
                continuer = False
            elif in_choice in ["hf1,rf2,hf3,rf4,hf5,rf6,hf7,tt8,hf9,tt10"]:
                robot_name = in_choice
                g_reset_world_client()
                continue

        except WrongException as inst:
            lg.debug(f"Exception catched: {inst.args[0]}")
            input()

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
    g_prompt_pub = rospy.Publisher("/simu_prompt", String, queue_size=1)
    g_head_cmd_pub = rospy.Publisher("/tiago_head_cmd", HeadCmd, queue_size=10)

    step_over_sub = rospy.Subscriber('/step_over', EmptyM, step_over_cb)
    human_visual_signal_sub = rospy.Subscriber('/human_visual_signals', Signal, human_visual_signal_cb)
    robot_visual_signal_sub = rospy.Subscriber('/robot_visual_signals', Signal, robot_visual_signal_cb)
    robot_visual_signal_pub = rospy.Publisher('/robot_visual_signals', Signal, queue_size=1)

    g_reset_world_client = rospy.ServiceProxy("/reset_world", EmptyS)
    g_hmi_timeout_max_client = rospy.ServiceProxy("hmi_timeout_max", Int)
    g_hmi_r_idle_client = rospy.ServiceProxy("hmi_r_idle", SetBool)
    g_go_idle_pose_client = rospy.ServiceProxy("go_idle_pose", EmptyS)
    g_go_home_pose_client = rospy.ServiceProxy("go_home_pose", EmptyS)
    start_human_action_service = rospy.Service("start_human_action", Int, start_human_action_server)
    ###
    
    # Execution simulation
    main_exec()
