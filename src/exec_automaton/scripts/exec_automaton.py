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
import simpleaudio as sa
import numpy as np

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

    print(f"Loading solution '{filename}' ... ", end="", flush=True)
    s_t = time.time()

    domain_name, pstates, final_pstates = dill.load(open(CM.path + filename, "rb"))

    print("Loaded! - %.2fs" %(time.time()-s_t))

    g_domain_name = domain_name
    CM.g_FINAL_PSTATES = final_pstates

    return pstates

def load_solution():
    """
    Loads the previously produced solution.
    The domain name is retreived and returned and as well as the solution tree and the initial step.
    """
    filename = "policy.p"
    return load(filename)

###############
## EXECUTION ##
###############
def wait_prompt_press_enter():
    global g_enter_pressed
    g_wait_press_enter_pub.publish(EmptyM())
    while not rospy.is_shutdown() and not g_enter_pressed:
        time.sleep(0.05)
    g_enter_pressed = False

def wait_prompt_button_pressed():
    global g_prompt_button_pressed
    g_show_prompt_button_client()
    while not rospy.is_shutdown() and not g_prompt_button_pressed:
        time.sleep(0.05)
    g_prompt_button_pressed = False

MAX_CHAR = 39 
def format_txt(s):

    words = s.split(" ")

    lines = []

    while words!=[]:
        line = ""
        while words!=[] and len(line) + len(words[0]) < MAX_CHAR:
            w = words.pop(0)
            if w=="\n":
                break
            line += " " + w
        
        lines.append(line)

    if LANG=="FR":
        i = lines[-1].find("(Suivant)")
    elif LANG=="ENG":
        i = lines[-1].find("(Next)")
    else:
        raise Exception("format_txt: LANG unknown")
    if i!=-1:
        l = lines[-1]
        while len(l)<MAX_CHAR:
            l = l[:i] + " " +l[i:]
            i+=1
        lines[-1] = l
    
    output_s = ""
    for l in lines:
        output_s += l + "\n"
    output_s = output_s[:-1]

    return output_s

def build_expected_ha(name, parameters):
    if name=="PASSIVE":
        HA = CM.Action.create_passive("H", "PASS")
    else:
        HA = CM.Action()
        HA.name = name
        HA.parameters = parameters
        HA.cost = 1.0
        HA.agent = "H" 

    return HA

sound_finished = sa.WaveObject.from_wave_file("/home/afavier/new_exec_sim_ws/src/exec_automaton/scripts/finished_70.wav")
def training():
    global TIMEOUT_DELAY, g_enter_pressed, g_force_exec_stop, TRAINING_PROMPT_ONLY, step_over


    # capabilities : pick, place cubes, drop, pass/TO
    # notion of steps: synchronized in step (sound), you and the robot can both perform one action or be passive. Either you or the robot act alone, or you act in parallel. Each step begins with a sound. You can perform only one action per step, so you have to wait the next step/sound before acting/clicking again. 
    # To be passive during a step you can either do nothing and wait for the robot to act, or click on the hand to indicate the robot you will be passive. Note that even if you're "passive", you can perform any valid action while the robot is acting. 

    back_TIMEOUT_DELAY = TIMEOUT_DELAY
    TIMEOUT_DELAY = TRAINING_TIMEOUT_DELAY

    TRAINING_PROMPT_ONLY = True

    curr_pstate = CM.g_PSTATES[0] #type: CM.PState
    look_at_human()

###############################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training0"][LANG]
    )))
    wait_prompt_button_pressed()

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training1"][LANG]
    )))
    g_show_tuto_zones_client()
    wait_prompt_button_pressed()

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training2"][LANG]
    )))
    wait_prompt_button_pressed()
    g_hide_tuto_zones_client()

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training3"][LANG]
    )))
    wait_prompt_button_pressed()

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training4"][LANG]
    )))
    wait_prompt_button_pressed()
    g_prompt_pub.publish(String(format_txt(
    #####################################  + 6 Lignes
    g_prompt_messages["training5"][LANG]
    )))

    expected_ha = build_expected_ha("pick", ("y1",))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha]) # IDLE to avoid pass
    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)

    result_id = MOCK_run_id_phase()
    RA = select_best_compliant_RA(curr_pstate, result_id)
    start_execute_RA(RA)

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training6"][LANG]
    )))

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)


###############################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training7"][LANG]
    )))
    wait_prompt_button_pressed()
    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training8"][LANG]
    )))

    expected_ha = build_expected_ha("place", ("l2", "y1"))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])
    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)
    
    RA = select_best_RA(curr_pstate)
    start_execute_RA(RA)

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training9"][LANG]
    )))

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training10"][LANG]
    )))
    wait_prompt_button_pressed()
    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training11"][LANG]
    )))
    wait_prompt_button_pressed()
    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training12"][LANG]
    )))

    send_NS_update_HAs(curr_pstate, VHA.NS, only_has=[])
    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)

    RA = select_best_RA_H_passive(curr_pstate)
    start_execute_RA(RA)

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training13"][LANG]
    )))

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training14"][LANG]
    )))
    wait_prompt_button_pressed()
    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training15"][LANG]
    )))
    wait_prompt_button_pressed()
    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training16"][LANG]
    )))

    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[])

    str_bar = IncrementalBarStr(max=TIMEOUT_DELAY, width=INCREMENTAL_BAR_STR_WIDTH)
    start_waiting_time = time.time()
    start_prompt_bar_pub.publish(EmptyM())
    time.sleep(0.01)
    while not rospy.is_shutdown() and time.time()-start_waiting_time<TIMEOUT_DELAY:
        str_bar.goto(time.time()-start_waiting_time)
        g_prompt_progress_bar_pub.publish(String(f"{str_bar.get_str()}"))
        time.sleep(0.01)
    str_bar.goto(str_bar.max)
    str_bar.finish()
    time.sleep(0.01)

    # Check if timeout reached
    timeout_reached = False 
    if g_new_human_decision==None:
        time.sleep(ESTIMATED_R_REACTION_TIME*1.1)
        if g_new_human_decision==None:
            timeout_reached = True
    if timeout_reached:
        g_prompt_pub.publish(String(format_txt(
        g_prompt_messages["training17"][LANG]
        )))
        sgl = Signal()
        sgl.type = Signal.TO
        robot_visual_signal_pub.publish(sgl)
        g_hmi_timeout_reached_pub.publish(EmptyM())

    RA = select_best_RA_H_passive(curr_pstate)
    start_execute_RA(RA)
    passive_update_HAs(curr_pstate, RA)
    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)


################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training18"][LANG]
    )))
    wait_prompt_button_pressed()

    RA = select_best_RA(curr_pstate)
    start_execute_RA(RA)

    expected_ha = build_expected_ha("pick", ("o1",))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training19"][LANG]
    )))

    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)
    step_over = False

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    RA = select_best_RA(curr_pstate)
    start_execute_RA(RA)

    expected_ha = build_expected_ha("place", ("l4", "o1"))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training20"][LANG]
    )))

    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)
    step_over = False

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training21"][LANG]
    )))
    wait_prompt_button_pressed()
    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training22"][LANG]
    )))


    expected_ha = build_expected_ha("pick", ("w1",))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])

    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)

    result_id = MOCK_run_id_phase()
    RA = select_best_compliant_RA(curr_pstate, result_id)
    start_execute_RA(RA)

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training23"][LANG]
    )))

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training24"][LANG]
    )))

    expected_ha = build_expected_ha("place", ("l6","w1"))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])

    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)

    RA = select_best_RA(curr_pstate)
    start_execute_RA(RA)

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training25"][LANG]
    )))

    RA = select_best_RA(curr_pstate)
    start_execute_RA(RA)

    expected_ha = build_expected_ha("pick", ("b2",))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])
    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)
    step_over = False

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training26"][LANG]
    )))

    send_NS_update_HAs(curr_pstate, VHA.NS, only_has=[])
    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)

    RA = select_best_RA_H_passive(curr_pstate)
    start_execute_RA(RA)

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training27"][LANG]
    )))

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training28"][LANG]
    )))

    go_idle_pose_once()
    RA = select_valid_passive(curr_pstate)

    expected_ha = build_expected_ha("drop", ("b2", "H"))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])
    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training29"][LANG]
    )))

    RA = select_valid_passive(curr_pstate)
    look_at_human()

    expected_ha = build_expected_ha("pick", ("p1",))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])
    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

################################################################

    RA = select_valid_passive(curr_pstate)
    look_at_human()

    expected_ha = build_expected_ha("place", ("l8", "p1"))
    send_NS_update_HAs(curr_pstate, VHA.NS_IDLE, only_has=[expected_ha])
    while not rospy.is_shutdown() and g_new_human_decision==None:
        time.sleep(0.01)

    wait_step_end()
    HA = MOCK_assess_human_action()
    curr_pstate = get_next_pstate(curr_pstate, HA, RA)
    reset()
    time.sleep(0.1)

###############################################################

    g_prompt_pub.publish(String(format_txt(
    g_prompt_messages["training30"][LANG]
    )))

    TRAINING_PROMPT_ONLY = False
    TIMEOUT_DELAY = back_TIMEOUT_DELAY
    log_event("OVER")
    g_force_exec_stop = True

    reset_head()
    g_go_idle_pose_client.call()
    sound_finished.play()
    wait_prompt_button_pressed()

    return -2, -2

def execution_HF():
    """
    Main algorithm 
    """
    global g_new_human_decision

    curr_pstate = CM.g_PSTATES[0] #type: CM.PState
    while not exec_over(curr_pstate) and not rospy.is_shutdown() and not g_force_exec_stop:

        print("\n")
        rospy.loginfo(f"Step {curr_pstate.id} begins.")
        

        if curr_pstate.isRInactive():
            prompt("HF_idle_step_started")
            send_NS_update_HAs(curr_pstate, VHA.NS_IDLE)
            look_at_human()
            go_idle_pose_once()
            RA = select_valid_passive(curr_pstate)
            wait_human_start_acting()

        else:
            go_home_pose_once()

            if check_if_human_is_done(curr_pstate):
                set_permanent_prompt_line("h_done")
                rospy.loginfo("You are done.")
            elif check_if_human_can_leave(curr_pstate):
                set_permanent_prompt_line("h_can_leave")
                rospy.loginfo("You can leave.")
            else:
                reset_permanent_prompt_line()

            send_NS_update_HAs(curr_pstate, VHA.NS, timeout=TIMEOUT_DELAY)

            if AUTO_PASS:
                g_new_human_decision = default_human_passive_action
                RA = select_best_RA_H_passive(curr_pstate)
            else:
                look_at_human()
                wait_human_decision(curr_pstate)

                ## 1 & 2 & 3 ##
                if human_active(): 
                    # reset_permanent_prompt_line()
                    ## 1 & 2 ##
                    if ID_needed(curr_pstate): 
                        result_id = MOCK_run_id_phase()
                        ## 1 ##
                        if ID_successful(result_id): 
                            RA = select_best_compliant_RA(curr_pstate, result_id)
                        ## 2 ##
                        else: 
                            RA = select_valid_passive(curr_pstate)
                    ## 3 ##
                    else: 
                        lg.debug("ID not needed.")
                        RA = select_best_RA(curr_pstate)
                ## 4 ##
                else: 
                    RA = select_best_RA_H_passive(curr_pstate)

            start_execute_RA(RA)
            passive_update_HAs(curr_pstate, RA)
                  
        wait_step_end()
        
        HA = MOCK_assess_human_action()

        # Check Passive Step
        if RA.is_passive() and HA.is_passive():
            pass # Repeat current step
        else:
            curr_pstate = get_next_pstate(curr_pstate, HA, RA)
        reset()
        time.sleep(0.1)

    # if forced stop
    if g_force_exec_stop:
        prompt("force_stop")
        reset_head()
        return -1, -1

    log_event("OVER")
    reset_permanent_prompt_line()
    prompt("task_done")
    reset_head()
    # g_go_init_pose_client.call()
    g_go_idle_pose_client.call()
    lg.info(f"END => {curr_pstate.id}")
    print(f"END => {curr_pstate.id}")
    g_hmi_finish_pub.publish(EmptyM())
    return int(curr_pstate.id)

def execution_RF():
    """
    Main algorithm 
    """
    global g_new_human_decision

    curr_pstate = CM.g_PSTATES[0] #type: CM.PState
    while not exec_over(curr_pstate) and not rospy.is_shutdown() and not g_force_exec_stop:

        rospy.loginfo(f"Step {curr_pstate.id} begins.")
        

        if curr_pstate.isRInactive():
            prompt("RF_idle_step_started")
            send_NS_update_HAs(curr_pstate, VHA.NS_IDLE)
            look_at_human()
            go_idle_pose_once()
            RA = select_valid_passive(curr_pstate)
            wait_human_start_acting()

        else:
            go_home_pose_once()
            
            if check_if_human_is_done(curr_pstate):
                set_permanent_prompt_line("h_done")
                rospy.loginfo("You are done.")
            elif check_if_human_can_leave(curr_pstate):
                set_permanent_prompt_line("h_can_leave")
                rospy.loginfo("You can leave.")
            else:
                reset_permanent_prompt_line()
            
            RA = select_best_RA(curr_pstate)

            if not RA.is_passive():
                send_NS(VHA.NS)
                compliant_pairs = find_compliant_pairs_with_RA(curr_pstate, RA)
                human_can_act_concurrently = False
                for p in compliant_pairs:
                    if not p.human_action.is_passive():
                        human_can_act_concurrently = True
                start_execute_RA(RA, rf=True, h_inactive=(not human_can_act_concurrently or AUTO_PASS))
                passive_update_HAs(curr_pstate, RA)

            elif RA.is_passive():
                look_at_human()
                send_NS_update_HAs(curr_pstate, VHA.NS, timeout=TIMEOUT_DELAY)
                if AUTO_PASS:
                    g_new_human_decision = default_human_passive_action
                    RA = select_best_active_RA(curr_pstate)
                else:
                    start_waiting_time = time.time()
                    str_bar = IncrementalBarStr(max = TIMEOUT_DELAY, width=INCREMENTAL_BAR_STR_WIDTH)
                    log_event("R_S_RF_WAIT_H")
                    prompt("rf_r_passif")
                    time.sleep(0.01)
                    start_prompt_bar_pub.publish(EmptyM())
                    time.sleep(0.01)
                    while not rospy.is_shutdown() and time.time()-start_waiting_time<str_bar.max and g_new_human_decision==None:
                        elapsed = time.time() - start_waiting_time
                        str_bar.goto(elapsed)
                        g_prompt_progress_bar_pub.publish(String(f"{str_bar.get_str()}"))
                        time.sleep(0.01)


                    if g_new_human_decision==None:
                        str_bar.goto(str_bar.max)
                        str_bar.finish()
                        g_prompt_progress_bar_pub.publish(String(f"{str_bar.get_str()}"))
                        g_hmi_timeout_reached_pub.publish(EmptyM())
                        rospy.loginfo("start wait reaction time")
                        start_time = time.time()
                        while not rospy.is_shutdown():
                            if not g_h_decision_received and time.time()-start_time>1.0:
                                break
                            elif g_h_decision_received and g_new_human_decision:
                                break
                            else:
                                time.sleep(0.01)
                        if g_new_human_decision==None or g_new_human_decision.is_passive():
                            RA = select_best_active_RA(curr_pstate)
                    elif g_new_human_decision.is_passive():
                        RA = select_best_active_RA(curr_pstate)

                passive_update_HAs(curr_pstate, RA)
                compliant_pairs = find_compliant_pairs_with_RA(curr_pstate, RA)
                human_can_act_concurrently = False
                for p in compliant_pairs:
                    if not p.human_action.is_passive():
                        human_can_act_concurrently = True
                log_event("R_E_RF_WAIT_H")
                start_execute_RA(RA, rf=True, h_inactive=(not human_can_act_concurrently or AUTO_PASS))

                  
        wait_step_end()
        
        HA = MOCK_assess_human_action()

        # Check Passive Step
        if RA.is_passive() and HA.is_passive():
            pass # Repeat current step
        else:
            curr_pstate = get_next_pstate(curr_pstate, HA, RA)
        reset()
        time.sleep(0.1)

    # if forced stop
    if g_force_exec_stop:
        prompt("force_stop")
        reset_head()
        return -1, -1

    log_event("OVER")
    prompt("task_done")
    reset_head()
    # g_go_init_pose_client.call()
    g_go_idle_pose_client.call()
    lg.info(f"END => {curr_pstate.id}")
    print(f"END => {curr_pstate.id}")
    g_hmi_finish_pub.publish(EmptyM())
    return int(curr_pstate.id)


#########################
## MOCK Human behavior ##
#########################
g_possible_human_actions = []
def send_NS_update_HAs(ps: CM.PState, type, timeout=0.0, only_has=None):
    """
    Send NextStep (NS) signal
    Update g_possible_human_actions
    Call send_vha
    """
    global g_possible_human_actions

    
    send_NS(type)

    # update g_possible_human_actions with passive action always at last index
    if only_has!=None:
        g_possible_human_actions = only_has
    else:
        poss_ha = []
        for ap in ps.children:
            # check if already in pos_ha:
            already = False
            for ha in poss_ha:
                if CM.Action.are_similar(ha, ap.human_action):
                    already = True
                    break
            if already:
                continue

            # else add it
            if ap.human_action.is_passive():
                poss_ha = poss_ha + [ap.human_action]
            else:
                poss_ha = [ap.human_action] + poss_ha
        g_possible_human_actions = poss_ha

    send_vha(g_possible_human_actions, type, timeout=timeout)

    # Find best human action id, sent to hmi mock
    # best_ha = Int32()
    # if step.best_human_pair.human_action.is_passive():
    #     best_ha.data = -1
    # else:
    #     for i,ho in enumerate(step.human_options):
    #         if CM.Action.are_similar(ho.human_action, step.best_human_pair.human_action):
    #             best_ha.data = i+1
    #             break
    # g_best_human_action_pub.publish(best_ha)

sound_ns = sa.WaveObject.from_wave_file("/home/afavier/new_exec_sim_ws/src/exec_automaton/scripts/sound.wav")
def send_NS(type, turn=None):
    sgl = Signal()
    if type==VHA.NS:
        sgl.type = Signal.NS
    elif type==VHA.NS_IDLE:
        sgl.type = Signal.NS_IDLE
    else:
        raise Exception("Invalid type to send NS signal")
    
    if turn==None:
        pass
    elif turn in [Signal.ROBOT_TURN, Signal.HUMAN_TURN]:
        sgl.turn = turn
    else:
        raise Exception("Invalid turn")
    robot_visual_signal_pub.publish(sgl)
    time.sleep(0.001)

    sound_ns.play()

def passive_update_HAs(ps: CM.PState, RA: CM.Action, timeout=0.0, only_has=None):
    global g_possible_human_actions

    if not human_active():
        if only_has!=None:
            g_possible_human_actions = only_has
        else:
            # find compliant human actions and send VHA
            compliant_pairs = find_compliant_pairs_with_RA(ps, RA)
            g_possible_human_actions = []
            for p in compliant_pairs:
                if p.human_action.is_passive():
                    g_possible_human_actions = g_possible_human_actions + [p.human_action]
                else:
                    g_possible_human_actions = [p.human_action] + g_possible_human_actions
        send_vha(g_possible_human_actions, VHA.CONCURRENT, timeout=timeout)

def find_compliant_pairs_with_RA(ps: CM.PState, RA):
    compliant_pairs = []
    for c in ps.children:
        if CM.Action.are_similar(RA, c.robot_action):
            compliant_pairs.append(c)
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
    rank_before = g_best_reachable_human_solution.get_f_leaf().getBestRank()
    rank_after = g_best_reachable_human_solution_after_robot_choice.get_f_leaf().getBestRank()

    lg.debug(f"#{rank_before} -> #{rank_after}")

    return rank_after>rank_before

##########################
## MOCK Robot execution ##
##########################
def MOCK_run_id_phase():
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
        id_result = g_new_human_decision
    else:
        rospy.loginfo("ID failed...")
        id_result = None
    
    log_event("R_E_ID")
    
    return id_result

def MOCK_assess_human_action() -> CM.Action:
    global g_new_human_decision
    # If Human didn't choose, we try to find passive human action
    log_event("R_S_ASSESS")

    if g_new_human_decision==None:
        for ha in g_possible_human_actions:
            if ha.is_passive():
                g_new_human_decision = ha
                break
        # If didn't find passive HA
        if g_new_human_decision==None: 
            rospy.loginfo("Inactive step...")
            g_new_human_decision = default_human_passive_action
    else:
        rospy.loginfo(f"Assessed Human action: {g_new_human_decision}")

    time.sleep(ASSESS_DELAY)
    log_event("R_E_ASSESS")
    
    return g_new_human_decision


#######################################
## SWITCH BETWEEN IDLE AND HOME POSE ##
#######################################
R_idle = False
def go_idle_pose_once():
    global R_idle
    if not R_idle:
        # reset_head()
        g_hmi_r_idle_client.call(True)
        g_go_idle_pose_client.call()
        R_idle = True
def go_home_pose_once():
    global R_idle
    if R_idle:
        prompt("out_of_idle")
        reset_head()
        g_hmi_r_idle_client.call(False)
        g_go_home_pose_client.call()
        R_idle = False

#######################
## Waiting Functions ##
#######################
def wait_human_start_acting():
    rospy.loginfo("Waiting for human to act...")
    log_event("R_S_WAIT_HSA")
    while not rospy.is_shutdown():
        if g_new_human_decision!=None:
            break
        time.sleep(0.1)

    log_event("R_E_WAIT_HSA")
    rospy.loginfo("Step start detected!")

def wait_human_decision(ps: CM.PState):
    log_event("R_S_WAIT_HC")

    rospy.loginfo("Waiting for human to act...")

    str_bar = IncrementalBarStr(max=TIMEOUT_DELAY, width=INCREMENTAL_BAR_STR_WIDTH)

    if not ps.isHInactive():
        start_waiting_time = time.time()
        prompt("wait_human_decision")
        time.sleep(0.01)
        start_prompt_bar_pub.publish(EmptyM())
        time.sleep(0.01)
        while not rospy.is_shutdown() and not ps.isHInactive() and time.time()-start_waiting_time<TIMEOUT_DELAY and g_new_human_decision==None:
            elapsed = time.time()-start_waiting_time

            # Update progress bars
            str_bar.goto(elapsed)
            g_prompt_progress_bar_pub.publish(String(f"{str_bar.get_str()}"))

            # Loop rate
            time.sleep(0.01)

        # Check if timeout reached
        if g_new_human_decision==None:
            str_bar.goto(str_bar.max)
            str_bar.finish()
            g_prompt_progress_bar_pub.publish(String(f"{str_bar.get_str()}"))
            rospy.loginfo("end loop")
            rospy.loginfo("start wait reaction time")
            start_time = time.time()
            while not rospy.is_shutdown():
                if not g_h_decision_received and time.time()-start_time>1.0:
                    break
                elif g_h_decision_received and g_new_human_decision:
                    break
                else:
                    time.sleep(0.01)
            rospy.loginfo("end wait reaction time")
        else:
            rospy.loginfo("end loop")
    
    # If Timeout Reached
    if g_new_human_decision==None:
        rospy.loginfo("Timeout reached, human seems passive...")
        prompt("wait_h_decision_timeout")
        sgl = Signal()
        sgl.type = Signal.TO
        robot_visual_signal_pub.publish(sgl)
        g_hmi_timeout_reached_pub.publish(EmptyM())
    # Visual signal received, either PASS or Start Action
    else:
        rospy.loginfo("Human decision detected!")
        if g_new_human_decision.is_pass():
            rospy.loginfo("PASS, Human is passive...")
        else:
            rospy.loginfo(f"Human is active")

    log_event("R_E_WAIT_HC")

def wait_step_end():
    log_event("R_S_WAIT_STEP_END")
    rospy.loginfo("Waiting step end...")
    while not rospy.is_shutdown() and not step_over:
        if not g_robot_acting:
            prompt("wait_end_ha")
        time.sleep(0.1)

    if human_active():
        # Because step_over isn't sent as a human visual signal....
        time.sleep(ESTIMATED_R_REACTION_TIME)

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
    return g_new_human_decision!=None and not g_new_human_decision.is_passive()

def ID_needed(ps: CM.PState):
    best_ra = None
    for c in ps.children:
        if c.best_compliant:
            if best_ra==None:
                best_ra = c.robot_action
            elif not CM.Action.are_similar(c.robot_action, best_ra):
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

def get_next_pstate(ps: CM.PState, HA: CM.Action, RA: CM.Action):
    executed_pair = None
    for c in ps.children:
        if CM.Action.are_similar(c.robot_action, RA) and CM.Action.are_similar(c.human_action, HA):
            executed_pair = c
    if executed_pair==None:
        raise Exception("get_next_pstate: executed pair not found!")
    return CM.g_PSTATES[executed_pair.child]

def convert_rank_to_score(rank, nb):
    return -1/(nb-1) * rank + nb/(nb-1)
    return rank

def reset():
    global g_possible_human_actions, g_new_human_decision, g_h_decision_received, step_over, g_previous_elapsed, g_robot_acting
    g_possible_human_actions = []
    g_new_human_decision = None
    g_h_decision_received = False
    step_over = False
    g_previous_elapsed = -1
    g_robot_acting = False

def full_reset():
    global g_force_exec_stop, g_best_reachable_human_solution, g_best_reachable_human_solution_after_robot_choice, g_enter_pressed, g_prompt_button_pressed, AUTO_PASS
    reset()
    reset_permanent_prompt_line()
    g_force_exec_stop = False
    g_best_reachable_human_solution = None
    g_best_reachable_human_solution_after_robot_choice = None
    g_enter_pressed = False
    g_prompt_button_pressed = False
    AUTO_PASS = False
    reset_auto_pass_hmi_pub.publish(EmptyM())

def get_first_step(begin_step: ConM.Step):
    if len(begin_step.children)!=1:
        raise Exception("begin_step should only have 1 child.")
    first_step = begin_step.children[0]
    return first_step

def get_agents_before_step(step: ConM.Step):
    return step.from_pair.end_agents

def exec_over(ps: CM.PState):
    return len(ps.children)==1 and ps.children[0].is_final()

def exec_over_tt(step):
    pairs = step.get_pairs()
    if len(pairs)==1 and (pairs[0].human_action.is_idle() or pairs[0].robot_action.is_idle()):
        if len(pairs[0].next)==1 and (pairs[0].next[0].human_action.is_idle() or pairs[0].next[0].robot_action.is_idle()):
            return True
    return False

def look_at_human():
    msg = HeadCmd()
    msg.type = HeadCmd.LOOK_AT_HUMAN
    g_head_cmd_pub.publish(msg)

def reset_head():
    msg = HeadCmd()
    msg.type = HeadCmd.RESET
    g_head_cmd_pub.publish(msg)

def check_if_human_can_leave(ps: CM.PState):
    # human can leave now if there is from the current step a sequence of pairs of actions leading to a final leaf
    # where the human is always passive
    for c in ps.children:
        if c.human_action.is_passive():
            if c.is_final() or (not c.is_passive() and check_if_human_can_leave(CM.g_PSTATES[c.child])):
                return True
    return False

def check_if_human_is_done(ps: CM.PState):
    # human is done if there is from the current step a sequence of pairs of actions leading to a final leaf
    # where the human is always IDLE
    for c in ps.children:
        if c.human_action.is_idle():
            if c.is_final() or (not c.is_passive() and check_if_human_is_done(CM.g_PSTATES[c.child])):
                return True
    return False

#########################
## Select Robot Action ##
#########################
def select_best_RA(ps: CM.PState) -> CM.Action:
    for c in ps.children:
        if c.best:
            return c.robot_action
    raise Exception("select_best_RA: best pair not found !")

def select_best_RA_H_passive(ps: CM.PState) -> CM.Action:

    for c in ps.children:
        if c.human_action.is_passive() and c.best_compliant:
            return c.robot_action
    # return default_robot_passive_action
    raise Exception("select_best_RA_H_passive: best h_pass compliant RA not found...")

def select_best_compliant_RA(ps: CM.PState, human_action: CM.Action) -> CM.Action:
    for c in ps.children:
        if c.best_compliant and CM.Action.are_similar(human_action, c.human_action):
            return c.robot_action 
    raise Exception("select_best_compliant_RA: best compliant robot action not found...")

def select_valid_passive(ps: CM.PState) -> CM.Action:
    # Find the first passive robot action
    for c in ps.children:
        if c.robot_action.is_passive():
            return c.robot_action
    raise Exception("select_valid_passive: Couldn't find a passive robot action")

def select_best_active_RA(ps: CM.PState) -> CM.Action:
    sorted_pairs = list(np.sort( np.array( ps.children ) ))
    for p in sorted_pairs:
        if not p.robot_action.is_passive():
            return p.robot_action
    raise Exception("select_best_active_RA: best active RA not found...")


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
def send_vha(valid_human_actions: list[CM.Action], type, timeout=0.0):
    """
    Send vha to hmi 
    Input is basically g_possible_human_actions
    Filter passive actions and send 
    """
    msg = VHA()
    msg.type = type
    msg.timeout = timeout

    # Remove passive actions
    for ha in valid_human_actions:
        if ha.is_passive():
            continue
        msg.valid_human_actions.append( ha.name + str(ha.parameters) )

    g_update_VHA_pub.publish(msg)

g_robot_acting = False
def start_execute_RA(RA: CM.Action, rf=False, h_inactive=False):
    global g_robot_acting
    rospy.loginfo(f"Execute Robot Action {RA}")

    if RA.is_passive():
        g_robot_acting = False
        prompt("robot_is_passive")
    else:
        g_robot_acting = True
        if rf:
            if h_inactive:
                prompt("rf_robot_acting_h_inactive")
            else:
                prompt("rf_robot_acting")
        else:
            prompt("robot_is_acting")

    msg = compute_msg_action(RA)
    g_robot_action_pub.publish(msg)
    time.sleep(0.01)

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
    if req.data==-10:
        raise Exception("action id = -10, should be impossible")
    elif req.data==-1:
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

g_new_human_decision = None # type: CM.Action | None
g_h_decision_received = False
def human_visual_signal_cb(msg: Signal):
    global g_new_human_decision, g_h_decision_received

    g_h_decision_received = True
    rospy.loginfo(f"CB human visual signal: id: {msg.id} type: {msg.type}")

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
        g_new_human_decision = pass_ha
        rospy.loginfo(f"Human visual : PASS")

    # Regular action
    elif msg.type == Signal.S_HA:
        g_new_human_decision = g_possible_human_actions[msg.id-1]
        rospy.loginfo(f"Human visual : S_HA")

def robot_visual_signal_cb(msg: Signal):
    global g_robot_acting
    if msg.type == Signal.E_RA:
        g_robot_acting = False
        rospy.loginfo("Robot action over.")

g_enter_pressed = False
def enter_pressed_cb(msg):
    global g_enter_pressed
    g_enter_pressed = True

g_prompt_button_pressed = False
def prompt_button_pressed_cb(msg):
    global g_prompt_button_pressed
    g_prompt_button_pressed = True

g_force_exec_stop = False
def force_exec_stop_cb(msg):
    global g_force_exec_stop
    g_force_exec_stop = True

############
## PROMPT ##
############

LANG = "FR" # ENG | FR
g_permanent_prompt = ""
g_prompt_start_extra = "  "
def set_permanent_prompt_line(msg_id):
    global g_permanent_prompt
    g_permanent_prompt = g_prompt_start_extra + g_prompt_messages[msg_id][LANG] + "\n\n"
def reset_permanent_prompt_line():
    global g_permanent_prompt
    g_permanent_prompt = ""
def prompt(msg_id: str, extra="", from_training=False):
    if not TRAINING_PROMPT_ONLY or (TRAINING_PROMPT_ONLY and from_training):
        g_prompt_pub.publish(String( g_permanent_prompt + g_prompt_start_extra + g_prompt_messages[msg_id][LANG] + extra ))
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
    "going_idle": {
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
        "FR":  "Réinitialisation...",
        },
    "h_done":{
        "ENG": "You are done.",
        "FR":  "Vous avez terminé.",
        },
    "h_can_leave":{
        "ENG": "If needed, I can finish alone.",
        "FR":  "Si besoin, je peux terminer seul.",
        },
    "human_turn":{
        "ENG": "It's your turn to act.",
        "FR":  "C'est à votre tour d'agir.",
        },
    "robot_turn":{
        "ENG": "It's my turn to act.",
        "FR":  "C'est à mon tour d'agir.",
        },
    "turn_idle":{
        "ENG": "I can't act.",
        "FR":  "Je ne peux pas agir.",
        },
    "TT_idle":{
        "ENG": "I let you proceed.",
        "FR":  "Je vous laisse faire.",
        },
    "start_ready":{
        "ENG": "\t* Ready to start *",
        "FR":  "\t* Prêt à démarrer *",
        },
    "start_press_enter":{
        "ENG": "Click on the yellow button",
        "FR":  "Cliquez sur le bouton jaune",
        },
    "force_stop":{
        "ENG": "Force Stop",
        "FR":  "Force Stop",
        },
    "h_can_t_act":{
        "ENG": "You can't act, I proceed.",
        "FR":  "Vous ne pouvez pas agir, je continue.",
        },
    "rf_r_passif":{
        "ENG": "I prefer you to act\n" + g_prompt_start_extra + "but I can act myself.",
        "FR":  "Je prefère que vous agissiez\n" + g_prompt_start_extra + "mais je peux agir moi même.",
        },
    "rf_robot_acting_h_inactive":{
        "ENG": "I'm performing an action",
        "FR":  "J'agis...",
        },
    "rf_robot_acting":{
        "ENG": "I'm performing an action\n" + g_prompt_start_extra + "You can act in parallel.",
        "FR":  "J'agis...\n" + g_prompt_start_extra + "Vous pouvez agir en même temps.",
        },
    "training":{
        "ENG": "TRAINING",
        "FR":  "TRAINING",
        },
    "h_instru_tee":{
        "ENG": " Objective:\n Finish the task as fast as possible.",
        "FR":  " Objectif:\n Finir la tâche au plus vite.",
        },
    "h_instru_hfe":{
        "ENG": " Objective:\n Be free as fast as possible.",
        "FR":  " Objectif:\n Être libéré au plus vite.",
        },
    "training0":{
        "ENG": "Welcome to this tutorial! \n \n In collaboration with the robot, you'll need to make the stack of cubes shown at the top left of the screen. (Next) Click on the button ⬇ ",
        "FR":  "Bienvenue dans ce tutoriel ! \n \n En collaboration avec le robot, vous allez devoir réaliser la pile de cube montrée en haut à gauche de l'écran. (Suivant) Cliquez sur le bouton ⬇ ",
        },
    "training1":{
        "ENG": "The table has different zones. \n \n On the right is the stacking area. \n \n On the left are the cubes arranged in three separate zones. (Next)",
        "FR":  "La table comporte différentes zones. A droite se trouve la zone d'empilement. \n A gauche se trouve les cubes disposés dans trois zones distinctes. (Suivant)",
        },
    "training2":{
        "ENG": "You can grab the cubes in front of you and those in the middle. \n Similarly, the robot can grab the cubes in front of itself and those in the middle. (Next)",
        "FR":  "Vous pouvez attraper les cubes disposés devant vous et ceux présent au milieu. \n De même, le robot peut attraper les cubes devant lui et ceux au milieu. (Suivant)",
        },
    "training3":{
        "ENG": "Note also that only cubes that can be placed immediately can be picked up. \n Thus, it is not possible to grab a cube in anticipation. (Next)",
        "FR":  "Notez également que seul les cubes pouvant être placés immédiatement peuvent être attrapés. \n Ainsi, il n'est pas possible d'attraper un cube à l'avance. (Suivant)",
        },
    "training4":{
        "ENG": "Now you need to understand the concept of steps. The start of a step is signaled by a beep from the robot. You and the robot can only perform a maximum of one action per step. (Next)",
        "FR":  "Il vous faut maintenant comprendre la notion d'étape. Le début d'une étape sera marqué par un signal sonore émis par le robot. Vous et le robot ne pouvez effectuer qu'une action maximum par étape. (Suivant)",
        },
    "training5":{
        "ENG": "The first step begins. \n \n To grab a cube, simply click on it. \n Grab the central yellow cube.",
        "FR":  "La première étape commence. \n \n Pour attraper un cube il vous suffit de cliquer dessus. \n Attrapez le cube jaune central.",
        },
    "training6":{
        "ENG": "The robot has observed your action and acts accordingly...",
        "FR":  "Le robot a observé votre action et agit en fonction en parallèle...",
        },
    "training7":{
        "ENG": "Your action and that of the robot are complete, so the step is finished. \n \n The next step is ready to start. (Next)",
        "FR":  "Votre action et celle du robot sont terminées, l'étape est donc terminée. \n La suivante est prête à démarrer. (Suivant)",
        },
    "training8":{
        "ENG": "The next step begins. \n \n To place a cube, simply click on the stacking area on the right. \n Place the yellow cube.",
        "FR":  "L'étape suivante commence. \n \n Pour placer un cube il vous suffit de cliquer à droite sur la zone d'empilement. \n Placez le cube jaune.",
        },
    "training9":{
        "ENG": "Once again, the robot has observed you and is following you, acting in parallel.",
        "FR":  "Une nouvelle fois, le robot vous a observé et vous suit en agissant en parallèle.",
        },
    "training10":{
        "ENG": "At each step you can choose to be passive and take no action. \n \n There are two ways of doing this. (Next)",
        "FR":  "A chaque étape vous pouvez choisir d'être passif et de ne pas agir. \n \n Il y a deux manières pour cela. (Suivant)",
        },
    "training11":{
        "ENG": "First, you can wave your hand. \n \n This explicitly indicates to the robot that you wish to be passive. (Next)",
        "FR":  "D'abord, vous pouvez faire un signe de la main. \n \n Cela indique explicitement au robot que vous souhaitez être passif. (Suivant)",
        },
    "training12":{
        "ENG": "The step begins. \n \n Click on the hand to wave and let the robot begin.",
        "FR":  "L'étape commence. \n \n Cliquez sur la main pour faire un signe et laisser le robot commencer.",
        },
    "training13":{
        "ENG": "The robot takes your signal into account and decides to grab its pink bar.",
        "FR":  "Le robot prend votre signal en compte et décide d'attraper sa barre rose.",
        },
    "training14":{
        "ENG": "Note that despite your hand signal, you could have decided to act in parallel with the robot. (Next)",
        "FR":  "Notez que malgré votre signe de la main vous auriez pu décider de tout de même agir en parallèle du robot. (Suivant)",
        },
    "training15":{
        "ENG": "At the start of a step, the robot can start a timer to wait for your decision. Without any action or sign from you, you'll be considered passive. (Next)",
        "FR":  "Au début d'une étape, le robot pourra lancer un chrono pour attendre votre décision. Sans action ni signe de votre part, vous serez considéré comme passif. (Suivant)",
        },
    "training16":{
        "ENG": "Wait for the timer to run out to let the robot act on its own.",
        "FR":  "Attendez la fin du chrono pour laisser le robot agir seul.",
        },
    "training17":{
        "ENG": "Without any indication, the robot has considered you passive and places its bar.",
        "FR":  "Sans indication, le robot vous a considéré comme passif et place sa barre.",
        },
    "training18":{
        "ENG": "The robot can also start, letting you act in parallel. \n In the next step, the robot will directly grab the grey cube. Meanwhile, grab the orange cube. (Next)",
        "FR":  "Le robot peut également commencer, vous laissant agir en parallèle. \n A la prochaine étape le robot va directement attraper le cube gris. Attrapez en parallèle le cube orange. (Suivant)",
        },
    "training19":{
        "ENG": "Grab the orange cube!",
        "FR":  "Attrapez le cube orange !",
        },
    "training20":{
        "ENG": "The next step has begun. \n \n Place the orange cube!",
        "FR":  "L'étape suivante a commencée. \n \n Placez le cube orange !",
        },
    "training21":{
        "ENG": "Notice here that the robot is able to place the blue cube, but must first remove the green cube. (Next)",
        "FR":  "Remarquez ici que le robot est capable de placer le cube bleu mais il doit d'abord enlever le cube vert. (Suivant)",
        },
    "training22":{
        "ENG": "The step begins. \n \n Grab the white cube to induce the robot to pick up the green cube.",
        "FR":  "L'étape commence. \n \n Attrapez le cube blanc pour inciter le robot à prendre le cube vert.",
        },
    "training23":{
        "ENG": "The robot starts to make its blue cube accessible.",
        "FR":  "Le robot commence à rendre son cube bleu accessible.",
        },
    "training24":{
        "ENG": "Place the white cube. Concurrently, the robot will place the green cube on the table.",
        "FR":  "Placez le cube blanc. En parallèle, le robot posera le cube vert sur table.",
        },
    "training25":{
        "ENG": "Now grab the blue cube while the robot grabs its own.",
        "FR":  "Attrapez maintenant le cube bleu pendant que le robot attrape le sien.",
        },
    "training26":{
        "ENG": "Finally, tell the robot that you're going to be passive.",
        "FR":  "Finalement, indiquez au robot que vous allez être passif.",
        },
    "training27":{
        "ENG": "Then the robot places its cube.",
        "FR":  "Le robot place alors son cube.",
        },
    "training28":{
        "ENG": "If, as now, you can no longer place your cube, you can place it back on the table by clicking on the left side of the table.",
        "FR":  "Si jamais, comme maintenant, vous ne pouvez plus placer votre cube, vous pouvez le reposer sur la table en cliquant sur la partie gauche de cette dernière.",
        },
    "training29":{
        "ENG": "Now you can complete the task by grabbing and placing your pink bar.",
        "FR":  "Pouvez maintenant terminer la tâche en attrapant puis en plaçant votre barre rose.",
        },
    "training30":{
        "ENG": "The task is complete, as is this tutorial. \n \n Thank you for your cooperation!",
        "FR":  "La tâche est terminée ainsi que ce tutoriel. \n \n Merci de votre coopération !",
        },
    "end_task":{
        "ENG": format_txt("\t *** Task complete *** \n \n Answer the questionnaire before clicking on the button to continue."),
        "FR":  format_txt("\t *** Tâche terminée *** \n \n Repondez au questionnaire puis cliquez sur le bouton pour continuer."),
        },
    "end_expe":{
        "ENG": "        *** Last task complete ***\n\n" + format_txt("Answer the questionnaire and the experiment will be over! Thanks!"),
        "FR":  "     *** Dernière tâche terminée ***\n\n" + format_txt("Repondez au questionnaire et l'expérience sera terminée ! Merci !"),
        },
    "tuto_wait_start":{
        "ENG": f"           *** Tutorial ***\n\n\n Click on the yellow button    ⬇ ",
        "FR":  f"           *** Tutoriel ***\n\n\n Cliquez sur le bouton jaune   ⬇ ",
        },

}

##########
## MAIN ##
##########
def asking_robot(robots):
    # Asking which robot to use?
    while True:
        robot_name = input("Which robot (or training)? ")
        if robot_name in robots:
            # Load correct policy and exec_regime
            exec_regime, policy_name, sol = robots[robot_name]
            if sol!=None:
                break
            else:
                print("Solution empty...")
        else:
            print("robot name unknown...")

    return robot_name

def wait_start_signal(robot_name, robots, i, h_instru):
    rospy.loginfo("READY TO START, waiting for start signal...")

    if robot_name=="t":
        g_prompt_pub.publish(String( g_prompt_messages["tuto_wait_start"][LANG] ))
    else:
        if LANG=="FR":
            g_prompt_pub.publish(String( f"    Scénario n°{i}    {robots[robot_name][0]}\n\n{g_prompt_messages[h_instru][LANG]}\n\n Cliquez sur le bouton jaune   ⬇ ") )
        elif LANG=="ENG":
            g_prompt_pub.publish(String( f"    Scenario n°{i}    {robots[robot_name][0]}\n\n{g_prompt_messages[h_instru][LANG]}\n\n Click on the yellow button    ⬇ ") )
    
    wait_prompt_button_pressed()
    reset_permanent_prompt_line()

def start_delay():
    # Start delay before beginning
    str_bar = IncrementalBarStr(max = START_SIMU_DELAY, width=INCREMENTAL_BAR_STR_WIDTH)
    start_time = time.time()
    prompt("start_simu_delay")
    time.sleep(0.01)
    start_prompt_bar_pub.publish(EmptyM())
    time.sleep(0.01)
    while not rospy.is_shutdown() and time.time()-start_time<START_SIMU_DELAY:
        elapsed = time.time() - start_time
        str_bar.goto(elapsed)
        g_prompt_progress_bar_pub.publish(String(f"{str_bar.get_str()}"))
        time.sleep(0.01)
    str_bar.goto(str_bar.max)
    str_bar.finish()
    g_prompt_progress_bar_pub.publish(String(f"{str_bar.get_str()}"))
    time.sleep(0.01)

AUTO_PASS = False
def auto_pass_state_cb(msg):
    global AUTO_PASS
    AUTO_PASS = msg.data

def repeat_loop(robot_name, robots):
    while True:
        in_choice = input("\nChoices:\n\t1- Repeat last\n\t2- Change Robot\n\t3- Training\n\t4- Stop\nAnswer: ")
        
        if in_choice=="1":
            return robot_name
        elif in_choice=="2":
            return asking_robot(robots)
        elif in_choice=="3":
            return "t"
        elif in_choice=="4":
            return None
        else:
            print("Unrecognized input, please answer again.")

def main_exec():
    global TIMEOUT_DELAY, ESTIMATED_R_REACTION_TIME, P_SUCCESS_ID_PHASE, ID_DELAY, ASSESS_DELAY, TT_R_PASSIVE_DELAY, TRAINING_TIMEOUT_DELAY
    global default_human_passive_action, default_robot_passive_action
    global INCREMENTAL_BAR_STR_WIDTH
    global TRAINING_PROMPT_ONLY
    global START_SIMU_DELAY
    global g_domain_name
    global g_enter_pressed
    global g_force_exec_stop

    # CONSTANTS #
    #   Delays 
    TIMEOUT_DELAY               = 4.0
    TRAINING_TIMEOUT_DELAY      = 3.0 #4.0
    ESTIMATED_R_REACTION_TIME   = 0.3
    ID_DELAY                    = 0.5
    ASSESS_DELAY                = 0.2
    TT_R_PASSIVE_DELAY          = 1.0
    START_SIMU_DELAY            = 2.0
    INCREMENTAL_BAR_STR_WIDTH   = 31
    #   Proba
    P_SUCCESS_ID_PHASE          = 1.0


    HUMAN_UPDATING = False

    TRAINING_PROMPT_ONLY = False

    time.sleep(0.5)

    # Init Seed
    seed = random.randrange(sys.maxsize)
    random.seed(seed)
    lg.debug(f"\nSeed was: {seed}")

    default_human_passive_action = CM.Action.create_passive("H", "PASS")
    default_robot_passive_action = CM.Action.create_passive("R", "PASS")

    ## LOADING ## # pstates
    policy_tee = load("policy_task_end_early.p")
    policy_hmw = load("policy_human_min_work.p")
    policy_fhfe = load("policy_fake_human_free_early.p")


    if g_domain_name!=DOMAIN_NAME:
        raise Exception("Missmatching domain names CONSTANT and loaded")
    robots = {
        "t" : ("training", "task_end_early", policy_tee, ""),

        "1" : ("Human-First", "task_end_early", policy_tee, "h_instru_tee"), # Finish task early
        "2" : ("Robot-First", "task_end_early", policy_tee, "h_instru_tee"), # Finish task early

        "3" : ("Human-First", "human_min_work", policy_hmw, "h_instru_tee"), # Finish task early
        "4" : ("Robot-First", "human_min_work", policy_hmw, "h_instru_tee"), # Finish task early

        "5" : ("Human-First", "fake_human_free_early", policy_fhfe, "h_instru_hfe"), # Be free early
        "6" : ("Robot-First", "fake_human_free_early", policy_fhfe, "h_instru_hfe"), # Be free early
    }

    rospy.loginfo("Wait for hmi to be started...")
    rospy.wait_for_service("hmi_started")
    rospy.loginfo("hmi started!")
    g_hmi_timeout_max_client(int(TIMEOUT_DELAY))

    rospy.loginfo("Waiting for reset_world service to be started...")
    rospy.wait_for_service("reset_world")
    rospy.loginfo("reset_world service started")

    rospy.loginfo("Waiting prompt to be started...")
    rospy.wait_for_service("prompt_started")
    rospy.loginfo("prompt started")

    exec_regime = None


    # given order
    order = []
    i = 7-len(order) # N° scenario
    if order!=[]:
        order = [str(o) for o in order]
    else:
        order.append(asking_robot(robots))
    while order!=[]:
        
        print("Order = ", order)
        robot_name = order.pop(0)
        exec_regime, policy_name, policy, h_instructions = robots[robot_name]
        CM.g_PSTATES = policy
        ConM.setPolicyName(policy_name)

        # Reset world
        prompt("reset_world")
        g_reset_world_client()

        # Wait for Start Signal from Prompt Window
        wait_start_signal(robot_name, robots, i, h_instructions)

        log_event(f"N{i}_{robot_name}_{exec_regime}_{h_instructions}")
        
        # Start delay before beginning
        start_delay()

        # Starting execution
        if exec_regime == "training":
            id = training()
        elif exec_regime == "Human-First":
            id = execution_HF()
        elif exec_regime == "Robot-First":
            id = execution_RF()
        else:
            raise Exception("unknown exec_regime.")
    
        # loop
        if id==-1: # force stop
            order = []
        elif id==-2: # training
            pass
        else: # nominal
            # nb_sol = len(begin_step.get_final_leaves())
            # r_score = convert_rank_to_score(r_rank,nb_sol)
            # print(f"END: id={id}, r_score=%.3f" % r_score)
            print("END, show trace ?")

        # Recap prompt
        recap=''
        if exec_regime!="training":
            if LANG=="ENG":
                if h_instructions=="h_instru_tee":
                    recap = "Finish Task Fast"
                elif h_instructions=="h_instru_hfe":
                    recap = "Be Free Fast"
            elif LANG=="FR":
                if h_instructions=="h_instru_tee":
                    recap =  "Finir Tâche Vite"
                elif h_instructions=="h_instru_hfe":
                    recap = "Être Libéré Vite"
            space = ""
            while len(exec_regime + space + recap)<MAX_CHAR:
                space += " "
            recap = exec_regime + space + recap + "\n\n" 

            sound_finished.play()
        if order==[]:
            g_prompt_pub.publish(String( recap + g_prompt_messages["end_expe"][LANG]))
        else:
            if exec_regime!='training':
                g_prompt_pub.publish(String( recap + g_prompt_messages["end_task"][LANG]))
                wait_prompt_button_pressed()
        full_reset()
        i+=1

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
    g_best_human_action_pub = rospy.Publisher('/mock_best_human_action', Int32, queue_size=1)
    g_event_log_pub = rospy.Publisher('/event_log', EventLog, queue_size=10)
    g_prompt_pub = rospy.Publisher("/simu_prompt", String, queue_size=1)
    g_prompt_progress_bar_pub = rospy.Publisher("/prompt_update_progress_bar", String, queue_size=1)
    g_head_cmd_pub = rospy.Publisher("/tiago_head_cmd", HeadCmd, queue_size=10)
    start_prompt_bar_pub = rospy.Publisher("/start_prompt_bar", EmptyM, queue_size=1)
    reset_auto_pass_hmi_pub = rospy.Publisher("/reset_auto_pass", EmptyM, queue_size=1)

    step_over_sub = rospy.Subscriber('/step_over', EmptyM, step_over_cb)
    human_visual_signal_sub = rospy.Subscriber('/human_visual_signals', Signal, human_visual_signal_cb)
    robot_visual_signal_sub = rospy.Subscriber('/robot_visual_signals', Signal, robot_visual_signal_cb)
    robot_visual_signal_pub = rospy.Publisher('/robot_visual_signals', Signal, queue_size=1)

    g_wait_press_enter_pub = rospy.Publisher("/wait_press_enter", EmptyM, queue_size=1)
    enter_pressed_sub = rospy.Subscriber('/enter_pressed', EmptyM, enter_pressed_cb)

    force_exec_stop_sub = rospy.Subscriber('/force_exec_stop', EmptyM, force_exec_stop_cb)
    
    auto_pass_state_sub = rospy.Subscriber('/auto_pass_state', Bool, auto_pass_state_cb)

    g_reset_world_client = rospy.ServiceProxy("reset_world", EmptyS)
    g_hmi_timeout_max_client = rospy.ServiceProxy("hmi_timeout_max", Int)
    g_hmi_r_idle_client = rospy.ServiceProxy("hmi_r_idle", SetBool)
    g_go_idle_pose_client = rospy.ServiceProxy("go_idle_pose", EmptyS)
    g_go_home_pose_client = rospy.ServiceProxy("go_home_pose", EmptyS)
    g_go_init_pose_client = rospy.ServiceProxy("go_init_pose", EmptyS)
    start_human_action_service = rospy.Service("start_human_action", Int, start_human_action_server)

    g_reset_last_click_client = rospy.ServiceProxy("reset_last_click", EmptyS)
    g_show_prompt_button_client = rospy.ServiceProxy("show_prompt_button", EmptyS)
    g_hide_prompt_button_client = rospy.ServiceProxy("hide_prompt_button", EmptyS)
    prompt_button_pressed_sub = rospy.Subscriber('/prompt_button_pressed', EmptyM, prompt_button_pressed_cb)

    g_show_tuto_zones_client = rospy.ServiceProxy("show_tuto_zones", EmptyS)
    g_hide_tuto_zones_client = rospy.ServiceProxy("hide_tuto_zones", EmptyS)

    # Wait for publisher init
    time.sleep(0.1)

    ###
    
    # Execution simulation
    main_exec()
