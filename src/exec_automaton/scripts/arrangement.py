#!/usr/bin/env python3
import sys
import os
from copy import deepcopy
import time

# sys.path.append("../hatpehda")
# sys.path.insert(0, "/home/afavier/ws/HATPEHDA/hatpehda")
sys.path.insert(0, "/home/afavier/exec_simulator_ws/src/exec_automaton/scripts")
import CommonModule as CM
import ConcurrentModule as ConM


######################################################
################### Primitive tasks ##################
######################################################

## pick_b ##
def o_pick_b_precond(state, agent):
    return state.pos_b.val=="table"
def o_pick_b_effects(state, agent):
    state.pos_b.val=agent
o_pick_b = CM.Operator("pick_b", pre_cond=o_pick_b_precond, effects=o_pick_b_effects)

## pick_r ##
def o_pick_r_precond(state, agent):
    return state.pos_r.val=="table"
def o_pick_r_effects(state, agent):
    state.pos_r.val=agent
o_pick_r = CM.Operator("pick_r", pre_cond=o_pick_r_precond, effects=o_pick_r_effects)

## pick_g ##
def o_pick_g_precond(state, agent):
    return state.pos_g.val=="table"
def o_pick_g_effects(state, agent):
    state.pos_g.val=agent
o_pick_g = CM.Operator("pick_g", pre_cond=o_pick_g_precond, effects=o_pick_g_effects)

## place_1 ##
def o_place_1_precond(state, agent):
    return nothing_at_place("1", state)
def o_place_1_effects(state, agent):
    if state.pos_b.val==agent:
        state.pos_b.val = "1"
    elif state.pos_r.val==agent:
        state.pos_r.val = "1"
    elif state.pos_g.val==agent:
        state.pos_g.val = "1"
    else:
        raise Exception("Holding nothing!!!")
o_place_1 = CM.Operator("place_1", pre_cond=o_place_1_precond, effects=o_place_1_effects)

## place_1 ##
def o_place_1_precond(state, agent):
    return nothing_at_place("1", state)
def o_place_1_effects(state, agent):
    if state.pos_b.val==agent:
        state.pos_b.val = "1"
    elif state.pos_r.val==agent:
        state.pos_r.val = "1"
    elif state.pos_g.val==agent:
        state.pos_g.val = "1"
    else:
        raise Exception("Holding nothing!!!")
o_place_1 = CM.Operator("place_1", pre_cond=o_place_1_precond, effects=o_place_1_effects)

## place_2 ##
def o_place_2_precond(state, agent):
    return nothing_at_place("2", state)
def o_place_2_effects(state, agent):
    if state.pos_b.val==agent:
        state.pos_b.val = "2"
    elif state.pos_r.val==agent:
        state.pos_r.val = "2"
    elif state.pos_g.val==agent:
        state.pos_g.val = "2"
    else:
        raise Exception("Holding nothing!!!")
o_place_2 = CM.Operator("place_2", pre_cond=o_place_2_precond, effects=o_place_2_effects)

## place_3 ##
def o_place_3_precond(state, agent):
    return nothing_at_place("3", state)
def o_place_3_effects(state, agent):
    if state.pos_b.val==agent:
        state.pos_b.val = "3"
    elif state.pos_r.val==agent:
        state.pos_r.val = "3"
    elif state.pos_g.val==agent:
        state.pos_g.val = "3"
    else:
        raise Exception("Holding nothing!!!")
o_place_3 = CM.Operator("place_3", pre_cond=o_place_3_precond, effects=o_place_3_effects)

## place_4 ##
def o_place_4_precond(state, agent):
    return nothing_at_place("4", state)
def o_place_4_effects(state, agent):
    if state.pos_b.val==agent:
        state.pos_b.val = "4"
    elif state.pos_r.val==agent:
        state.pos_r.val = "4"
    elif state.pos_g.val==agent:
        state.pos_g.val = "4"
    else:
        raise Exception("Holding nothing!!!")
o_place_4 = CM.Operator("place_4", pre_cond=o_place_4_precond, effects=o_place_4_effects)

## Drink ##
def o_drink_precond(state, agent):
    return state.thirsty.val==True
def o_drink_effects(state, agent):
    state.thirsty.val = False
o_drink = CM.Operator("drink", pre_cond=o_drink_precond, effects=o_drink_effects)

## pushing ##
def o_pushing_effects(state, agent):
    state.pos_b.val="3_bis"
o_pushing = CM.Operator("pushing", effects=o_pushing_effects)

common_ops = [o_pick_b, o_pick_r, o_pick_g, o_place_1, o_place_2, o_place_3, o_place_4]
robot_ops = common_ops + [o_pushing]
human_ops = common_ops + [o_drink]


######################################################
################### Abstract Tasks ###################
######################################################

## Arrange ##
def m_Arrange_decomp(state, agent):
    if is_one_placed_and_other_holding_by(CM.g_other_agent_name[agent], state) or is_two_placed(state):
        return []
    return [("Pick",), ("Place",), ("Arrange",)]
m_Arrange = CM.Method("Arrange", decomp=m_Arrange_decomp)

# Without repeat..
def m_Arrange0_decomp(state, agent):
    return [("Pick",), ("Place",)]
m_Arrange0 = CM.Method("Arrange", decomp=m_Arrange0_decomp)

# With drink..
def m_Arrange1_decomp(state, agent):
    if is_one_placed_and_other_holding_by(CM.g_other_agent_name[agent], state):
        if state.thirsty.val:
            return [("drink",)]
        else:
            return []
    return [("drink",), ("Pick",), ("Place",), ("Arrange",)]
m_Arrange1 = CM.Method("Arrange", decomp=m_Arrange1_decomp)

def m_Arrange2_decomp(state, agent):
    if is_one_placed_and_other_holding_by(CM.g_other_agent_name[agent], state):
        if state.thirsty.val:
            return [("drink",)]
        else:
            return []
    return [("Pick",), ("Place",), ("drink",), ("Arrange",)]
m_Arrange2 = CM.Method("Arrange", decomp=m_Arrange2_decomp)

## Pick ##
def m_Pick_b_precond(state, agent):
    return state.pos_b.val=="table"
def m_Pick_b_decomp(state, agent):
    return [("pick_b",)]
m_Pick_b = CM.Method("Pick", pre_cond=m_Pick_b_precond, decomp=m_Pick_b_decomp)

def m_Pick_r_precond(state, agent):
    return state.pos_r.val=="table"
def m_Pick_r_decomp(state, agent):
    return [("pick_r",)]
m_Pick_r = CM.Method("Pick", pre_cond=m_Pick_r_precond, decomp=m_Pick_r_decomp)

def m_Pick_g_precond(state, agent):
    return state.pos_g.val=="table"
def m_Pick_g_decomp(state, agent):
    return [("pick_g",)]
m_Pick_g = CM.Method("Pick", pre_cond=m_Pick_g_precond, decomp=m_Pick_g_decomp)

## Place ##
def m_Place_precond(state, agent):
    return state.pos_b.val==agent or state.pos_r.val==agent or state.pos_g.val==agent
def m_Place_decomp(state, agent):
    if state.pos_b.val==agent:
        return [("Place_b",)]
    elif state.pos_r.val==agent:
        return [("Place_r",)]
    elif state.pos_g.val==agent:
        return [("Place_g",)]
m_Place = CM.Method("Place", pre_cond=m_Place_precond, decomp=m_Place_decomp)

## Place_b1 ##
def m_Place_b1_precond(state, agent):
    return state.pos_b.val==agent and nothing_at_place("1", state)
def m_Place_b1_decomp(state, agent):
    return [("place_1",)]
m_Place_b1 = CM.Method("Place_b", pre_cond=m_Place_b1_precond, decomp=m_Place_b1_decomp)

## Place_b3 ##
def m_Place_b3_precond(state, agent):
    return state.pos_b.val==agent and nothing_at_place("3", state)
def m_Place_b3_decomp(state, agent):
    return [("place_3",)]
m_Place_b3 = CM.Method("Place_b", pre_cond=m_Place_b3_precond, decomp=m_Place_b3_decomp)

## Place_r2 ##
def m_Place_r2_precond(state, agent):
    return state.pos_r.val==agent and nothing_at_place("2", state)
def m_Place_r2_decomp(state, agent):
    return [("place_2",)]
m_Place_r2 = CM.Method("Place_r", pre_cond=m_Place_r2_precond, decomp=m_Place_r2_decomp)

## Place_r3 ##
def m_Place_r3_precond(state, agent):
    return state.pos_r.val==agent and nothing_at_place("3", state)
def m_Place_r3_decomp(state, agent):
    return [("place_3",)]
m_Place_r3 = CM.Method("Place_r", pre_cond=m_Place_r3_precond, decomp=m_Place_r3_decomp)

## Place_g3 ##
def m_Place_g3_precond(state, agent):
    return state.pos_g.val==agent and nothing_at_place("3", state)
def m_Place_g3_decomp(state, agent):
    return [("place_3",)]
m_Place_g3 = CM.Method("Place_g", pre_cond=m_Place_g3_precond, decomp=m_Place_g3_decomp)

## Place_g4 ##
def m_Place_g4_precond(state, agent):
    return state.pos_g.val==agent and nothing_at_place("4", state)
def m_Place_g4_decomp(state, agent):
    return [("place_4",)]
m_Place_g4 = CM.Method("Place_g", pre_cond=m_Place_g4_precond, decomp=m_Place_g4_decomp)

## Place_blocked ##
def m_Place_g_block_precond(state, agent):
    return state.pos_g.val==agent and not nothing_at_place("3", state)
def m_Place_g_block_decomp(state, agent):
    return [("pushing",), ("Place_g",)]
m_Place_g_block = CM.Method("Place_g", pre_cond=m_Place_g_block_precond, decomp=m_Place_g_block_decomp)

common_methods = [("Place",m_Place), ("Place_r",m_Place_r2)]
robot_methods = common_methods + [("Pick",m_Pick_r,m_Pick_g), ("Place_g",m_Place_g3,m_Place_g_block), ("Arrange",m_Arrange0)]
human_methods = common_methods + [("Pick",m_Pick_b,m_Pick_r), ("Place_b",m_Place_b1,m_Place_b3), ("Arrange",m_Arrange0)]

# common_methods = [("Place",m_Place), ("Place_r",m_Place_r2,m_Place_r3)]
# robot_methods = common_methods + [("Pick",m_Pick_r,m_Pick_g), ("Place_g",m_Place_g4), ("Arrange",m_Arrange)]
# human_methods = common_methods + [("Pick",m_Pick_b,m_Pick_r), ("Place_b",m_Place_b1), ("Arrange",m_Arrange)]


######################################################
###################### Triggers ######################
######################################################

common_triggers = []
robot_triggers = common_triggers + []
human_triggers = common_triggers + []


######################################################
###################### Helpers #######################
######################################################

def nothing_at_place(place, state):
    return state.pos_b.val!=place and state.pos_r.val!=place and state.pos_g.val!=place

def is_one_placed_and_other_holding_by(agent, state):
    if state.pos_b.val=="1":
        if state.pos_b.val==agent or state.pos_r.val==agent or state.pos_g.val==agent:
            return True
        if state.pos_r.val=="2" or state.pos_r.val=="3":
            return True
        elif state.pos_g.val=="4":
            return True
    elif state.pos_r.val=="2" or state.pos_r.val=="3":
        if state.pos_b.val==agent or state.pos_r.val==agent or state.pos_g.val==agent:
            return True
        if state.pos_g.val=="4":
            return True
    elif state.pos_g.val=="4":
        if state.pos_b.val==agent or state.pos_r.val==agent or state.pos_g.val==agent:
            return True
    return False

def is_two_placed(state):
    if state.pos_b.val=="1":
        if state.pos_r.val=="2" or state.pos_g.val=="3":
            return True
    if state.pos_r.val=="2":
        if state.pos_b.val=="1" or state.pos_g.val=="3":
            return True
    if state.pos_g.val=="3":
        if state.pos_b.val=="1" or state.pos_r.val=="2":
            return True
    return False

######################################################
######################## MAIN ########################
######################################################

def initDomain():
    # Set domain name
    domain_name = os.path.basename(__file__)[:-3]
    CM.set_domain_name(domain_name)

    # Set agents name
    robot_name = "robot"
    human_name = "human"
    CM.set_agents_name(robot_name=robot_name, human_name=human_name)

    # Initial state
    initial_state = CM.State("init")

    # Static properties
    initial_state.create_static_fluent("self_name", "None")
    initial_state.create_static_fluent("other_agent_name", {robot_name:human_name, human_name:robot_name}) 

    # Dynamic properties
    initial_state.create_dyn_fluent("pos_b", "table")
    initial_state.create_dyn_fluent("pos_r", "table")
    initial_state.create_dyn_fluent("pos_g", "table")
    initial_state.create_dyn_fluent("thirsty", True)

    # Robot init #
    CM.declare_operators(robot_name, robot_ops)
    CM.declare_methods(robot_name, robot_methods)
    # htpa.declare_triggers(robot_name, *robot_triggers)
    robot_state = deepcopy(initial_state)
    robot_state.__name__ = robot_name + "_init"
    robot_state.self_name.val = robot_name
    CM.set_state(robot_name, robot_state)
    CM.add_tasks(robot_name, [("Arrange",)])

    # Human init #
    CM.declare_operators(human_name, human_ops)
    CM.declare_methods(human_name, human_methods)
    human_state = deepcopy(initial_state)
    human_state.__name__ = human_name + "_init"
    human_state.self_name.val = human_name
    CM.set_state(human_name, human_state)
    CM.add_tasks(human_name, [("Arrange",)])

    # Starting Agent
    CM.set_starting_agent(robot_name)

def main():
    initDomain()
    sol = ConM.explore()
    return sol

if __name__ == "__main__":
    # CM.DebugPrinter.disable()
    # sys.stdout = CM.DevNull()
    sol = main()
    ConM.show_solution(sol)
    ConM.dumping_solution(sol)