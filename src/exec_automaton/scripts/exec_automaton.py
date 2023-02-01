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

class IdResult(Enum):
    NOT_NEEDED=0
    FAILED=1

DEBUG = False
INPUT = False
########
# DEBUG = True
# INPUT = True

sys.path.insert(0, "/home/afavier/exec_simulator_ws/src/exec_automaton/scripts")
# sys.path.insert(0, "/home/afavier/ws/HATPEHDA/hatpehda")
import ConcurrentModule as ConM
# import importlib

from arrangement import *
# from conflict_pick import *
# from simple import *

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

    rate = rospy.Rate(10) # 10hz

    curr_step = get_first_step(begin_step)
    while not rospy.is_shutdown() and not exec_over(curr_step):
        wait_step_start(curr_step)
        MOCK_H_action_choice(curr_step)

        if is_ID_needed(curr_step):
            result_id = MOCK_run_id_phase(curr_step)
            if ID_successful(result_id):
                RA = pick_any_valid_RA(curr_step, result_id)
            else:
                RA = curr_step.SRA
        else:
            lg.debug("ID not needed.")
            result_id = IdResult.NOT_NEEDED
            RA = curr_step.SRA

        MOCK_execute_RA(RA)

        MOCK_H_LRDe(curr_step, RA)
        HA = MOCK_assess_human_action(result_id)
        curr_step = get_next_step(curr_step, HA, RA)
        wait_step_end()
    lg.debug(f"END => {curr_step}")
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
            lg.debug("Human only let robot decide")
            valid_pairs = []
            for p in step.get_pairs():
                if p.human_action.name!="LRD" and CM.Action.are_similar(p.robot_action, RA):
                    valid_pairs.append(p)
            if len(valid_pairs)>0:
                HC = random.choice(valid_pairs).human_action


##########################
## MOCK Robot execution ##
##########################
def MOCK_execute_RA(RA: CM.Action):
    lg.debug(f"Execute Robot Action {RA}")

def MOCK_run_id_phase(step: ConM.Step):
    """
    Simulate the identification phase.
    LRD are always identified.
    For other human actions, the ID phase has a P_SUCCESS_ID_PHASE chance to succeed.
    If successful, there is a P_WRONG_ID chance that the ID the actually wrong.
    """
    lg.debug("Start ID phase...")
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
            lg.debug("ID failed...")
            id_result = IdResult.FAILED

    if id_result!=IdResult.FAILED:
        lg.debug(f"ID successful: {id_result}")
    
    return id_result

def MOCK_assess_human_action(result_id):
    lg.debug(f"Assessed Human action: {HC}")
    if result_id!=IdResult.NOT_NEEDED and result_id.name!="LRD" and not CM.Action.are_similar(result_id, HC):
        lg.debug("Robot was wrong about Human action...")
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
    # i.e. wait for human to start acting
    lg.debug("\nCurrent step:")
    lg.debug(CM.str_agents(get_agents_before_step(step)))
    lg.debug(step.str())
    lg.debug("Waiting for human to act...")
    if not DEBUG:
        if INPUT:
            input()

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

def pick_any_valid_RA(step: ConM.Step, human_action: CM.Action):
    # Pick robot action according to identified human action
    # And apply this action

    for ho in step.human_options:
        if ho.human_action==human_action:
            human_option = ho
            break
    
    robot_actions = human_option.robot_actions
    robot_action = robot_actions[0]
    if len(robot_actions)!=1 or robot_action.name!="WAIT":
        if robot_action.name=="WAIT" and len(robot_actions)>1:
            robot_action = robot_actions[1]

    return robot_action

def wait_step_end():
    lg.debug("Waiting step end...")
    if not DEBUG:
        if INPUT:
            input()

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


##########
## MAIN ##
##########
def show_solution_exec():
    ConM.render_tree(begin_step)

def main_exec(domain_name, solution_tree, begin_step):
    global P_SUCCESS_ID_PHASE, P_WRONG_ID, P_LET_ROBOT_DECIDE, P_LEAVE_ROBOT_DO

    # Mock ID phase
    P_SUCCESS_ID_PHASE  = 0.6
    P_WRONG_ID          = 0.2
    # Mock human behavior (set to -1 to make it equiprobable) (must have P_LRDe + P_LRDo <= 1)
    P_LEAVE_ROBOT_DO    = 0.4
    P_LET_ROBOT_DECIDE  = 0.1

    # Init Seed
    seed = random.randrange(sys.maxsize)
    random.seed(seed)
    lg.debug(f"\nSeed was: {seed}")

    initDomain()

    if INPUT:
        print("Press Enter to start...")
        input()

    try:
        result = execution_simulation(begin_step)
    except WrongException as inst:
        lg.debug(f"Exception catched: {inst.args[0]}")
        result=-1

    return (result, seed)

if __name__ == "__main__":
    rospy.init_node("exec_automaton")
    domain_name, solution_tree, begin_step = load_solution()
    main_exec(domain_name, solution_tree, begin_step)