#!/usr/bin/env python3
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
from std_msgs.msg import Int32, Bool
from std_msgs.msg import Empty as EmptyM
from progress.bar import IncrementalBar
from sim_msgs.srv import Int, IntResponse
from std_srvs.srv import Empty as EmptyS
from std_srvs.srv import SetBool
from sim_msgs.msg import Signal

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.collections import PolyCollection
import numpy as np
import matplotlib.ticker as ticker
import math
import time

import matplotlib.patches as mpatches

from sim_msgs.msg import EventLog
from datetime import datetime
import tkinter as tk
from tkinter import filedialog

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)
import ConcurrentModule as ConM
import CommonModule as CM

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')

# Event(name: str, stamp: float)
class Event:
    def __init__(self, name: str, stamp: float) -> None:
        self.name = name
        self.stamp = stamp

class LogSignal:
    def __init__(self, name: str, stamp: float, type, color_arrow, color_text) -> None:
        self.name = name
        self.stamp = stamp
        self.type = type
        self.color_arrow = color_arrow
        self.color_text = color_text

# Activity(name: str, t_s: float, t_e: float)
class Activity:
    def __init__(self, name, t_s, t_e) -> None:
        self.name = name
        self.t_s = t_s
        self.t_e = t_e
        self.h_is_best = False

    def dur(self):
        return self.t_e - self.t_s

g_events = [] #type: List[Event]

g_r_activities = [] #type: List[Activity]
g_h_activities = [] #type: List[Activity]

g_r_signals = [] #type: List[LogSignal]
g_h_signals = [] #type: List[LogSignal]
g_to_signals = [] #type: List[LogSignal]


def reset_times():
    global g_events, g_h_signals, g_r_signals, g_to_signals

    if len(g_events)==0:
        raise Exception("Events empty...")

    g_events.sort(key=getStamp)

    # look for first NS event
    i = 0
    while g_events[i].name[:len("SGL_NS")] != "SGL_NS":
        i+=1

    # save first time
    min_time = g_events[i].stamp

    # update all other events
    for x in g_events + g_r_signals + g_h_signals + g_to_signals:
        x.stamp -= min_time

def getStamp(event):
    return event.stamp

# Events Callback
def log_cb(msg):
    global g_events
    print(f"EVENT {msg.timestamp:.3f} {msg.name}")
    g_events.append( Event(msg.name, time.time()) )

# Signals Callbacks (Robot, Human, Timeouts)
## signals create a corresponding global event  
r_sgl_color_arrow = "lightskyblue"
r_sgl_color_text = "black"
g_r_signals_names = { # sgl_types : [display_name, shape_color, text_color]
    Signal.NS :         ["NS",      r_sgl_color_arrow, r_sgl_color_text],
    Signal.NS_IDLE :    ["NS_IDLE", r_sgl_color_arrow, r_sgl_color_text],
    Signal.S_RA :       ["S_RA",    r_sgl_color_arrow, r_sgl_color_text],
    Signal.E_RA :       ["E_RA",    r_sgl_color_arrow, r_sgl_color_text],
    Signal.R_PASS :     ["R_PASS",  r_sgl_color_arrow, r_sgl_color_text],
}
h_sgl_color_arrow = "wheat"
h_sgl_color_text = "black"
g_h_signals_names = { # sgl_types : [display_name, shape_color, text_color]
    Signal.S_HA :   ["S_HA",    h_sgl_color_arrow, h_sgl_color_text],
    Signal.E_HA :   ["E_HA",    h_sgl_color_arrow, h_sgl_color_text],
    Signal.H_PASS : ["H_PASS",  h_sgl_color_arrow, h_sgl_color_text],
}
def r_sgl_cb(sgl: Signal):
    global g_r_signals, g_to_signals

    if sgl.type == Signal.TO:
        t = time.time()
        TO_name = "TIMEOUT" 
        g_to_signals.append( LogSignal(TO_name, t, sgl.type, None, None) )
        g_events.append( Event("SGL_"+TO_name, t) )
        
        print(f"SGL_R {t:.3f} {TO_name} type={sgl.type}")

    elif sgl.type in g_r_signals_names:
        # t = rospy.get_time()
        t = time.time()
        g_r_signals.append( LogSignal(g_r_signals_names[sgl.type][0], t, sgl.type, g_r_signals_names[sgl.type][1], g_r_signals_names[sgl.type][2]) )
        event_name = g_r_signals_names[sgl.type][0]
        if sgl.turn == Signal.ROBOT_TURN:
            event_name += "_ROBOT_TURN"
        elif sgl.turn == Signal.HUMAN_TURN:
            event_name += "_HUMAN_TURN"
        g_events.append( Event("SGL_"+event_name, t) )

        print(f"SGL_R {t:.3f} {g_r_signals_names[sgl.type][0]} type={sgl.type} id={sgl.id}")

def h_sgl_cb(sgl: Signal):
    global g_h_signals

    if sgl.type in g_h_signals_names:
        t = time.time()
        g_h_signals.append( LogSignal(g_h_signals_names[sgl.type][0], t, sgl.type, g_h_signals_names[sgl.type][1], g_h_signals_names[sgl.type][2]) )
        g_events.append( Event("SGL_"+g_h_signals_names[sgl.type][0], t) )

        print(f"SGL_H {t:.3f} {g_h_signals_names[sgl.type][0]} type={sgl.type} id={sgl.id}")

# Activities Extraction
g_r_activities_names = { 
    # act_name :            [display_name,                  shape_color,            text_color]
    "rf_wait_h":            ["Wait H",                      "yellow",               "black"],
    "wait_hc":              ["Wait H",                      "yellow",               "black"],
    "plan_mvt":             ["Plan Motion",                    "silver",               "black"],
    "idle_wait_h":          ["Passive\nWait HA",            "silver",               "black"],
    "grns":                 ["GRNS",                        "silver",               "black"],
    "id":                   ["ID Phase",                    "lightgreen",           "black"],
    "sa":                   ["SA",                          "lightgreen",           "black"],
    "wait_end_step":        ["Wait End",               "forestgreen",          "white"],
    "passive":              ["Pass",                        "lightgrey",           "black"],
    "wait_turn":            ["Wait Turn",                   "lightgrey",           "black"],
}
g_h_activities_names = { 
    # act_name :            [display_name,                  shape_color,            text_color]
    "start_delay":          ["Decision",                    "yellow",               "black"],
    "passive":              ["Passive\nNo Signal",          "lightgrey",            "black"],
    "pass":                 ["Passive\nWith Signal",        "lightgrey",            "black"],
    "wait_ns":              ["Wait NS",                     "forestgreen",          "white"],
    "wait_turn":            ["Wait Turn",                   "lightgrey",            "black"],
}
def r_extract_activities():
    global g_events, g_r_activities

    g_r_activities = []

    i = 0
    e = g_events[i]
    while e.name[:6]!="SGL_NS":
        i+=1
        e = g_events[i]
    

    # Turn Taking #
    if e.name[-len("ROBOT_TURN"):] in ["ROBOT_TURN", "HUMAN_TURN"]:
        while True:

            last_act_name = "grns"

            if e.name=="SGL_NS_ROBOT_TURN":
                i+=1
                e = g_events[i]

                while e.name!="SGL_R_PASS" and e.name[:4]!="S_RA":
                    i+=1
                    e = g_events[i]

                if e.name=="SGL_R_PASS":
                    t2 = e.stamp
                    last_act_name = "passive"

                elif e.name[:4]=="S_RA":
                    t5 = e.stamp
                    j_split = e.name.find("(")
                    action_name = e.name[5:j_split] + "\n" + e.name[j_split:]

                    while e.name!="SGL_S_RA":
                        i+=1
                        e = g_events[i]
                    t4 = e.stamp
                    g_r_activities.append( Activity("plan_mvt", t5, t4) )

                    while e.name!="SGL_E_RA":
                        i+=1
                        e = g_events[i]
                    t3 = e.stamp
                    g_r_activities.append( Activity(action_name, t4, t3) )

                    while e.name!="R_E_WAIT_STEP_END":
                        i+=1
                        e = g_events[i]
                    t2 = e.stamp
                    g_r_activities.append( Activity("wait_end_step", t3, t2) )

            elif e.name in ["SGL_NS_HUMAN_TURN", "SGL_NS_IDLE_HUMAN_TURN"]:
                t4 = e.stamp
                i+=1
                e = g_events[i]

                while e.name!="R_S_ASSESS":
                    i+=1
                    e = g_events[i]
                t3 = e.stamp
                g_r_activities.append( Activity("wait_turn", t4, t3) )

                while e.name!="R_E_ASSESS":
                    i+=1
                    e = g_events[i]
                t2 = e.stamp
                g_r_activities.append( Activity("sa", t3, t2) )

            elif e.name == "SGL_NS_IDLE_ROBOT_TURN":
                t2 = e.stamp
                i+=1
                e = g_events[i]
                last_act_name = "passive"

            while e.name[:6]!="SGL_NS" and e.name!="OVER":
                i+=1
                e = g_events[i]
            t1 = e.stamp
            g_r_activities.append( Activity(last_act_name, t2, t1) )

            if e.name == "OVER":
                break
    
    # HF or RF
    else:
        while True:

            if e.name == "SGL_NS":

                while e.name!="R_S_RF_WAIT_H" and e.name[:4]!="S_RA" and e.name!="R_S_WAIT_HC":
                    i+=1
                    e = g_events[i]

                if e.name=="R_S_RF_WAIT_H":
                    t9 = e.stamp

                    while e.name!="R_E_RF_WAIT_H":
                        i+=1
                        e = g_events[i]
                    t8 = e.stamp
                    g_r_activities.append( Activity("rf_wait_h", t9, t8) )

                    while e.name!="R_S_WAIT_STEP_END" and e.name[:4]!="S_RA":
                        i+=1
                        e = g_events[i]

                    if e.name=="R_S_WAIT_STEP_END":
                        t5 = e.stamp

                if e.name=="R_S_WAIT_HC":
                    t10 = e.stamp
                    
                    while e.name!="R_E_WAIT_HC":
                        i+=1
                        e = g_events[i]
                    t8 = e.stamp
                    g_r_activities.append( Activity("wait_hc", t10, t8) )

                    while e.name[:4]!="S_RA" and e.name!="SGL_R_PASS" and e.name!="R_E_ID":
                        i+=1
                        e = g_events[i]

                    if e.name=="R_E_ID":
                        g_r_activities.append( Activity("id", t8, e.stamp))
                        t8 = e.stamp

                        while e.name[:4]!="S_RA" and e.name!="SGL_R_PASS":
                            i+=1
                            e = g_events[i]
                    
                    if e.name=="SGL_R_PASS":
                        t5 = e.stamp
                        g_r_activities.append( Activity("plan_mvt", t8, t5) )


                if e.name[:4]=="S_RA":
                    t7 = e.stamp
                    j_split = e.name.find("(")
                    action_name = e.name[5:j_split] + "\n" + e.name[j_split:]
                    
                    while e.name!="SGL_S_RA":
                        i+=1
                        e = g_events[i]
                    t6 = e.stamp
                    g_r_activities.append( Activity("plan_mvt", t7, t6) )

                    while e.name!="SGL_E_RA":
                        i+=1
                        e = g_events[i]
                    t5 = e.stamp
                    g_r_activities.append( Activity(action_name, t6, t5) )

            elif e.name == "SGL_NS_IDLE":
                t6 = e.stamp

                while e.name!="R_E_WAIT_HSA":
                    i+=1
                    e = g_events[i]
                t5 = e.stamp
                g_r_activities.append( Activity("idle_wait_h", t6, t5) )

            while e.name!="R_E_WAIT_STEP_END":
                i+=1
                e = g_events[i]
            t4 = e.stamp
            g_r_activities.append( Activity("wait_end_step", t5, t4) )

            while e.name!="R_S_ASSESS":
                i+=1
                e = g_events[i]
            t3 = e.stamp

            while e.name!="R_E_ASSESS":
                i+=1
                e = g_events[i]
            t2 = e.stamp
            g_r_activities.append( Activity("sa", t3, t2) )

            while e.name[:6]!="SGL_NS" and e.name!="OVER":
                i+=1
                e = g_events[i]
            t1 = e.stamp
            g_r_activities.append( Activity("grns", t2, t1) )

            if e.name == "OVER":
                break
def h_extract_activities():
    global g_events, g_h_activities

    g_h_activities = []

    i=0
    e = g_events[i]

    while e.name[:6]!="SGL_NS":
        i+=1
        e = g_events[i]
    

    # Turn Taking #
    if e.name[-len("ROBOT_TURN"):] in ["ROBOT_TURN", "HUMAN_TURN"]:
        while True:

            last_act_name = "passive"

            if e.name in ["SGL_NS_ROBOT_TURN", "SGL_NS_IDLE_ROBOT_TURN"]:
                t2 = e.stamp
                i+=1
                e = g_events[i]
                last_act_name = "wait_turn"

            elif e.name in ["SGL_NS_HUMAN_TURN", "SGL_NS_IDLE_HUMAN_TURN"]:
                t2 = e.stamp
                i+=1
                e = g_events[i]

                while e.name[:6]!="SGL_NS" and e.name!="OVER" and e.name[:4]!="S_HA" and e.name!="SGL_H_PASS":
                    i+=1
                    e = g_events[i]

                if e.name[:4]=="S_HA":
                    t3 = e.stamp
                    last_act_name = "wait_ns"
                    j_split = e.name.find("(")
                    action_name = e.name[5:j_split] + "\n" + e.name[j_split:]
                    g_h_activities.append( Activity("start_delay", t2, t3) )

                    while e.name!="SGL_E_HA":
                        i+=1
                        e = g_events[i]
                    t2 = e.stamp
                    g_h_activities.append( Activity(action_name, t3, t2) )

                elif e.name=="SGL_H_PASS":
                    last_act_name = "wait_ns"
                    g_h_activities.append( Activity("start_delay", t2, e.stamp) )
                    t2 = e.stamp

            while e.name[:6]!="SGL_NS" and e.name!="OVER":
                i+=1
                e = g_events[i]
            t1 = e.stamp
            g_h_activities.append( Activity(last_act_name, t2, t1) )

            if e.name == "OVER":
                break
    
    # HF or RF
    else:
        while True:

            last_act_name = "passive"
            before_s_ha_name = "start_delay"

            if e.name=="SGL_NS_IDLE":
                t2 = e.stamp
                i+=1
                e = g_events[i]

                while e.name[:4]!="S_HA":
                    i+=1
                    e = g_events[i]

            elif e.name == "SGL_NS":
                t2 = e.stamp
                i+=1
                e = g_events[i]

                while e.name[:6]!="SGL_NS" and e.name!="OVER" and e.name[:4]!="S_HA" and e.name!="SGL_H_PASS":
                    i+=1
                    e = g_events[i]

                if e.name=="SGL_H_PASS":
                    g_h_activities.append( Activity("start_delay", t2, e.stamp))
                    t2 = e.stamp
                    before_s_ha_name = "pass"

                    while e.name[:4]!="S_HA" and e.name[:6]!="SGL_NS" and e.name!="OVER":
                        i+=1
                        e = g_events[i]
                    last_act_name = "pass"
                    

            if e.name[:4]=="S_HA":
                last_act_name = "wait_ns"
                t3 = e.stamp
                j_split = e.name.find("(")
                action_name = e.name[5:j_split] + "\n" + e.name[j_split:]
                g_h_activities.append( Activity(before_s_ha_name, t2, t3))

                while e.name!="SGL_E_HA":
                    i+=1
                    e = g_events[i] 
                t2 = e.stamp
                g_h_activities.append( Activity(action_name, t3, t2))
                
            while e.name[:6]!="SGL_NS" and e.name!="OVER":
                i+=1
                e = g_events[i]
            t1 = e.stamp
            g_h_activities.append( Activity(last_act_name, t2, t1) )

            if e.name == "OVER":
                break

# Display
def show_events(events):
    for e in events:
        print(f"{e.name} - {e.stamp:.2f}")
def show_signals(signals):
    for s in signals:
        print(f"{s.name}_{s.type} - {s.stamp:.2f}")
def show_activities(activities):
    for a in activities:
        name = a.name.replace('\n','')
        print(f"{name} - {a.t_s:.2f} > {a.t_e:.2f}")

def extract_nb_optimal_h_action():
    nb_optimal_h_action = 0

    i_r = 0
    i_h = 0
    ps = CM.g_PSTATES[0]
    while i_r<len(g_r_activities) and i_h<len(g_h_activities):

        ## IDENTIFY EXECUTED PAIR ##
        # identify HA 
        if g_h_activities[i_h].name=="passive":
            ha_name = "Passive"
        elif g_h_activities[i_h].name=="start_delay":

            i_h+=1
            if g_h_activities[i_h].name=="pass":

                i_h+=1 
                if i_h>=len(g_h_activities) or g_h_activities[i_h].name in ["start_delay", "passive"]:
                    ha_name = "Passive"
                    i_h-=1

                elif g_h_activities[i_h].name not in g_h_activities_names:
                    ha_name = g_h_activities[i_h].name.replace('\n','')
                    i_h+=1

            elif g_h_activities[i_h].name not in g_h_activities_names:
                ha_name = g_h_activities[i_h].name.replace('\n','')
                i_h+=1

        i_h+=1 # next_step
    
        # identify RA
        while g_r_activities[i_r].name != "wait_end_step":
            i_r+=1
        if g_r_activities[i_r-1].name not in g_r_activities_names:
            ra_name = g_r_activities[i_r-1].name.replace('\n','')
        else:
            ra_name = "Passive"
        i_r+=3 # next_step
    
        # find executed pair in graph
        found = False
        for p in ps.children:
            p_ha_name = p.human_action.name.capitalize() + "("
            for k in p.human_action.parameters:
                p_ha_name += k + ","
            p_ha_name = p_ha_name[:-1] + ")"
            if p_ha_name[:len("Drop")]=="Drop":
                p_ha_name = "DropCube" + p_ha_name[len("Drop"):-3] + ")"

            p_ra_name = p.robot_action.name.capitalize() + "("
            for k in p.robot_action.parameters:
                p_ra_name += k + ","
            p_ra_name = p_ra_name[:-1] + ")"
            if p_ra_name[:len("Drop")]=="Drop":
                p_ra_name = "DropCube" + p_ra_name[len("Drop"):-3] + ")" # Due to mismatch in sim_controller overiding Drop event name...

            if p_ra_name==ra_name or (ra_name=="Passive" and p_ra_name[:len("Passive")]=="Passive"):
                if p_ha_name==ha_name or (ha_name=="Passive" and p_ha_name[:len("Passive")]=="Passive"):
                    found = True
                    break
        if not found:
            raise Exception("Corresponding pair not found...")
        executed_pair = p
        del p, found, p_ha_name, p_ra_name

        # extract executed_ha_name
        executed_ha_name = executed_pair.human_action.name.capitalize() + "("
        for k in executed_pair.human_action.parameters:
            executed_ha_name += k + ","
        del k
        executed_ha_name = executed_ha_name[:-1] + ")"
        if executed_ha_name[:len("Drop")]=="Drop":
            executed_ha_name = "DropCube" + executed_ha_name[len("Drop"):-3] + ")"

        # extract executed_ra_name
        executed_ra_name = executed_pair.robot_action.name.capitalize() + "("
        for k in executed_pair.robot_action.parameters:
            executed_ra_name += k + ","
        del k
        executed_ra_name = executed_ra_name[:-1] + ")"
        if executed_ra_name[:len("Drop")]=="Drop":
            executed_ra_name = "DropCube" + executed_ra_name[len("Drop"):-3] + ")"


        ####################

        ## ACCORDING TO REGIME, CHECK IF HA IS OPTIMAL

        # in Human-First, optimal human policy is HA of pair with best, optimal robot policy is RA of pair with HA and best_compliant
        # in Robot-First, optimal human policy is HA of pair with best_compliant_h, optimal robot policy is RA of pair with best

        ha_is_optimal = False

        regime = g_events[0].name[len('ni_i_'):len('ni_i_')+len('human-first')]
        if regime=="Human-First":
            # find best pair
            for p in ps.children:
                if p.best:
                    break
            best_pair = p
            del p

            # get best_ha_name
            best_ha_name = best_pair.human_action.name.capitalize() + "("
            for k in best_pair.human_action.parameters:
                best_ha_name += k + ","
            del k
            best_ha_name = best_ha_name[:-1] + ")"
            if best_ha_name[:len("Drop")]=="Drop":
                best_ha_name = "DropCube" + best_ha_name[len("Drop"):-3] + ")"

            # Compare 
            ha_is_optimal = best_ha_name==executed_ha_name

        elif regime=="Robot-First":
            # find best compliant pair
            for p in ps.children:
                p_ra_name = p.robot_action.name.capitalize() + "("
                for k in p.robot_action.parameters:
                    p_ra_name += k + ","
                p_ra_name = p_ra_name[:-1] + ")"
                if p_ra_name[:len("Drop")]=="Drop":
                    p_ra_name = "DropCube" + p_ra_name[len("Drop"):-3] + ")" # Due to mismatch in sim_controller overiding Drop event name...

                if p_ra_name==executed_ra_name and p.best_compliant_h:
                    break
            best_compliant_h_pair = p
            del p

            # get best_compliant_ha_name
            best_compliant_ha_name = best_compliant_h_pair.human_action.name.capitalize() + "("
            for k in best_compliant_h_pair.human_action.parameters:
                best_compliant_ha_name += k + ","
            del k
            best_compliant_ha_name = best_compliant_ha_name[:-1] + ")"
            if best_compliant_ha_name[:len("Drop")]=="Drop":
                best_compliant_ha_name = "DropCube" + best_compliant_ha_name[len("Drop"):-3] + ")"

            # Compare
            ha_is_optimal = best_compliant_ha_name==executed_ha_name

        else:
            raise Exception("Execution regiment not found...")

        ############################
    
        if ha_is_optimal:
            nb_optimal_h_action+=1
            if ha_name=="Passive":
                g_h_activities[i_h-1].h_is_best = True
            else:
                g_h_activities[i_h-2].h_is_best = True
            
        # update current pstate
        ps = CM.g_PSTATES[executed_pair.child]

    return nb_optimal_h_action

def extract_metrics():
    metrics = {}

    # task completion time
    metrics["task_completion_time"] = g_events[-1].stamp

    # number of steps
    metrics["number_steps"] = 0
    for sgl in g_r_signals:
        if sgl.type in [Signal.NS, Signal.NS_IDLE]:
            metrics["number_steps"] += 1

    # decision time - total / average / SD
    decision_time_a = []
    for a in g_h_activities:
        if a.name == "start_delay":
            decision_time_a.append(a.dur())
    decision_time_a = np.array(decision_time_a)
    metrics["decision_time_total"]    = np.sum(decision_time_a)
    metrics["decision_time_min"]      = np.min(decision_time_a)
    metrics["decision_time_max"]      = np.max(decision_time_a)
    metrics["decision_time_average"]  = np.mean(decision_time_a)
    metrics["decision_time_sd"]       = np.std(decision_time_a)

    # wait ns - total / average / SD
    wait_ns_a = []
    for a in g_h_activities:
        if a.name == "wait_ns":
            wait_ns_a.append(a.dur())
    wait_ns_a = np.array(wait_ns_a)
    metrics["wait_ns_total"]    = np.sum(wait_ns_a)
    metrics["wait_ns_min"]      = np.min(wait_ns_a)
    metrics["wait_ns_max"]      = np.max(wait_ns_a)
    metrics["wait_ns_average"]  = np.mean(wait_ns_a)
    metrics["wait_ns_sd"]       = np.std(wait_ns_a)

    # h action time - total / average / SD
    h_actions_a = []
    for a in g_h_activities:
        if a.name not in g_h_activities_names:
            h_actions_a.append(a.dur())
    h_actions_a = np.array(h_actions_a)
    metrics["h_action_nb"]              = len(h_actions_a)
    metrics["h_action_time_total"]      = np.sum(h_actions_a)
    metrics["h_action_time_min"]        = np.min(h_actions_a)
    metrics["h_action_time_max"]        = np.max(h_actions_a)
    metrics["h_action_time_average"]    = np.mean(h_actions_a)
    metrics["h_action_time_sd"]         = np.std(h_actions_a)


    # r action time - total / average / SD
    r_actions_a = []
    for a in g_r_activities:
        if a.name not in g_r_activities_names:
            r_actions_a.append(a.dur())
    metrics["r_action_nb"]              = len(r_actions_a)
    metrics["r_action_time_total"]      = np.sum(r_actions_a)
    metrics["r_action_time_min"]        = np.min(r_actions_a)
    metrics["r_action_time_max"]        = np.max(r_actions_a)
    metrics["r_action_time_average"]    = np.mean(r_actions_a)
    metrics["r_action_time_sd"]         = np.std(r_actions_a)


    ## Nb_h_optimal_action
    metrics["nb_h_optimal_action"] = extract_nb_optimal_h_action()
    
    ## Ratio_h_optimal_action
    metrics["ratio_h_optimal_action"] = 100 * metrics["nb_h_optimal_action"]/metrics["number_steps"]

    ## Time_human_free
    for h_act in g_h_activities:
        if h_act.name == "Place\n(l3,p1)":
            break
    metrics['time_human_free'] = h_act.t_e

    ## mvt plan
    plan_mvt_a = []
    for a in g_r_activities:
        if a.name == "plan_mvt":
            plan_mvt_a.append(a.dur())
    plan_mvt_a = np.array(plan_mvt_a)
    metrics["plan_mvt_total"]    = np.sum(plan_mvt_a)
    metrics["plan_mvt_min"]      = np.min(plan_mvt_a)
    metrics["plan_mvt_max"]      = np.max(plan_mvt_a)
    metrics["plan_mvt_average"]  = np.mean(plan_mvt_a)
    metrics["plan_mvt_sd"]       = np.std(plan_mvt_a)

    return metrics

def show_metric_line(k):
    if isinstance(metrics[k], int):
        print(f"\t-{k}: {metrics[k]}")
    else:
        print(f"\t-{k}: {metrics[k]:.2f}")

def show_metrics(metrics):
    print("Metrics:")
    for k in metrics:
        show_metric_line(k)
        
def excel_format_show_metrics(metrics):
    print(metrics["task_completion_time"], end=" ")
    print(metrics["number_steps"], end=" ")
    print(metrics["nb_h_optimal_action"], end=" ")
    print(metrics["ratio_h_optimal_action"], end=" ")
    print(metrics["decision_time_total"], end=" ")
    print(metrics["decision_time_average"], end=" ")
    print(metrics["decision_time_sd"], end=" ")
    print(metrics["wait_ns_total"], end=" ")
    print(metrics["wait_ns_average"], end=" ")
    print(metrics["wait_ns_sd"], end=" ")
    print(metrics["h_action_nb"], end=" ")
    print(metrics["h_action_time_total"], end=" ")
    print(metrics["h_action_time_average"], end=" ")
    print(metrics["h_action_time_sd"], end=" ")
    print(metrics["r_action_nb"], end=" ")
    print(metrics["r_action_time_total"], end=" ")
    print(metrics["r_action_time_average"], end=" ")
    print(metrics["r_action_time_sd"], end=" ")
    print(metrics["time_human_free"], end=" ")

def get_n_scenario(f):
    return f[1]

##########
## MAIN ##
##########

def load(filename, verbose=True):
    global g_domain_name

    if verbose:
        print(f"Loading solution '{filename}' ... ", end="", flush=True)
        s_t = time.time()

    domain_name, pstates, final_pstates = dill.load(open(CM.path + filename, "rb"))

    if verbose:
        print("Loaded! - %.2fs" %(time.time()-s_t))

    g_domain_name = domain_name
    CM.g_FINAL_PSTATES = final_pstates
    CM.g_PSTATES = pstates

def get_stamp(e):
    return e.stamp

if __name__ == "__main__":
    sys.setrecursionlimit(100000)

    sys.argv.append("load")
    # sys.argv.append("edit")
    record = sys.argv[1] == "record"
    edit = sys.argv[1] == "edit"

    path = "/home/afavier/new_exec_sim_ws/events/"

    # ROS Startup
    rospy.init_node('timelog')

    # Subscribers
    log_sub = rospy.Subscriber('/event_log', EventLog, log_cb)
    log_r_sgl_sub = rospy.Subscriber('/robot_visual_signals', Signal, r_sgl_cb)
    log_h_sgl_sub = rospy.Subscriber('/human_visual_signals', Signal, h_sgl_cb)


    ##########
    ## EDIT ##
    ##########
    if edit:
        # Get file names
        root = tk.Tk()
        root.withdraw()
        input_files = list(filedialog.askopenfilenames(initialdir=path, filetypes=(("dumped files","*.p"),) ))
        file_path = input_files[0]

        # Loading
        (g_events, g_r_signals, g_h_signals, g_to_signals) = dill.load(open(file_path, "rb"))
        if g_events[0].name[-len("training_"):]=="training_":
            raise Exception("Cannot show the timelog of the training task!")

        i_s = file_path.find("instru_")+len("instru_")
        i_e = i_s + 3
        h_instru = file_path[i_s:i_e]
        if h_instru=="tee":
            load("policy_task_end_early.p", verbose=False)
        elif h_instru=="hfe":
            load("policy_real_human_free_early.p", verbose=False)
        else:
            raise Exception("h_instru unknown...")
        
        # Edition:

        # 32.65
        # must be after NS around 32.8, around 32.85
        # offset of 0.228
        # must move event start S_HA g_events[59]
        # must move sgl S_HA g_h_signals[6]  and associate EVENT SGL_S_HA g_events[60]
        
        offset = 0.228
        g_events[59].stamp += offset
        g_events[60].stamp += offset
        g_h_signals[6].stamp = g_events[60].stamp


        g_events.sort(key=get_stamp)

        # Dumping
        dill.dump((g_events, g_r_signals, g_h_signals, g_to_signals), open(path + "edited_events.p", "wb"))
        print("edited events dumped")

        exit()

    ############
    ## RECORD ##
    ############
    if record:
        while not rospy.is_shutdown():
            print("\nStart recording")

            # wait start
            while len(g_events)==0 or g_events[-1].name=="OVER":
                time.sleep(0.1)

            # wait over
            while g_events[-1].name!="OVER":
                time.sleep(0.1)
            
            # Dumping
            dill.dump((g_events, g_r_signals, g_h_signals, g_to_signals), open(path + "events.p", "wb"))
            str_date = datetime.now().strftime("%d-%m-%Y_%H:%M:%S")
            dill.dump((g_events, g_r_signals, g_h_signals, g_to_signals), open(path + str_date + "_" + g_events[0].name + "_events.p", "wb"))
            print("events dumped")

            # Clearing
            g_events.clear()
            g_r_signals.clear()
            g_h_signals.clear()
            g_to_signals.clear()

    #############
    ## LOADING ##
    #############
    else:

        # Get file names
        root = tk.Tk()
        root.withdraw()
        input_files = list(filedialog.askopenfilenames(initialdir=path, filetypes=(("dumped files","*.p"),) ))

        ######################
        # Multiple files
        if len(input_files)>1:
            files = []
            for f in input_files:
                i_n = f.find('_N')
                n_scenario = f[i_n+4]
                files.append( (f, n_scenario) )
            files.sort(key=get_n_scenario)

            for f in files:
                file_path = f[0]
                ################################
        ########## LOADING EVENTS AND SIGNALS ##
                ################################

                # Loading
                (g_events, g_r_signals, g_h_signals, g_to_signals) = dill.load(open(file_path, "rb"))
                if g_events[0].name[-len("training_"):]=="training_":
                    raise Exception("Cannot show the timelog of the training task!")

                i_s = file_path.find("instru_")+len("instru_")
                i_e = i_s + 3
                h_instru = file_path[i_s:i_e]
                if h_instru=="tee":
                    load("policy_task_end_early.p", verbose=False)
                elif h_instru=="hfe":
                    load("policy_real_human_free_early.p", verbose=False)
                else:
                    raise Exception("h_instru unknown...")

                ###################
        ########## EXTRACT INFOS ##
                ###################

                reset_times()
                r_extract_activities()
                h_extract_activities()
                excel_format_show_metrics(extract_metrics())

            print("\n")

        ######################
        # Single file
        else:
            
            file_path = input_files[0]

            ################################
    ########## LOADING EVENTS AND SIGNALS ##
            ################################

            # Loading
            (g_events, g_r_signals, g_h_signals, g_to_signals) = dill.load(open(file_path, "rb"))
            if g_events[0].name[-len("training_"):]=="training_":
                raise Exception("Cannot show the timelog of the training task!")
            print("events loaded")

            i_s = file_path.find("instru_")+len("instru_")
            i_e = i_s + 3
            h_instru = file_path[i_s:i_e]
            if h_instru=="tee":
                load("policy_task_end_early.p")
            elif h_instru=="hfe":
                load("policy_real_human_free_early.p")
            else:
                raise Exception("h_instru unknown...")

            ########################
    ########## EXTRACT ACTIVITIES ##
            ########################

            # Reset time of event w.r.t. first NS signal
            reset_times()

            # Show Events
            print("\nEVENTS:")
            show_events(g_events)
            
            # Extract and Show Robot Activities
            r_extract_activities()
            print("\nROBOT ACTIVITIES:")
            show_activities(g_r_activities)

            # Extract and Show Human Activities
            h_extract_activities()
            print("\nHUMAN ACTIVITIES:")
            show_activities(g_h_activities)

            # Show SIGNALS
            print("\nR SIGNALS:")
            show_signals(g_r_signals)
            print("\nH SIGNALS:")
            show_signals(g_h_signals)
            print("\nTO SIGNALS:")
            show_signals(g_to_signals)

            # Show run informations (Robot type, id)
            print("\nRUN:")
            print(g_events[0].name)

            #####################
    ########## COMPUTE METRICS ##
            #####################

            metrics = extract_metrics()
            show_metrics(metrics)

            ##################
    ########## SHOW TIMELOG ##
            ##################

            MIN_DURATION = 0.15
            activities_zorder = 1
            ns_lines_zorder = 2
            signal_arrow_zorder = 3
            signal_text_zorder = 10
            signal_width_text = 1.5
            signal_style="Simple, head_width=8, head_length=4, tail_width=4"

            ####################
            WITH_TIMEOUT = True
            ####################


            plt.rcParams.update({'font.size': 13})

            fig, ax = plt.subplots(figsize=(18, 5))
            ax.invert_yaxis()
            
            ax.set_axisbelow(True)
            ax.grid(color='lightgrey', linestyle='dashed', axis='x')
            tick_spacing = 1
            horizontal_space = 0.2
            max_x = g_events[-1].stamp
            ax.set_xlim((-horizontal_space,max_x+horizontal_space))
            x_ticks = np.arange(0, max_x, tick_spacing )
            ax.set_xticks(x_ticks)
            ax.set_xticklabels(["" for x in x_ticks], rotation=90)
            ax.set_yticks([0.25, 1.0, 2.0, 3.0])
            ax.set_yticklabels(['TO', 'R', 'Signals', 'H'], rotation=90)
            ax.set_ylim(3.5, 0.0)
            if not WITH_TIMEOUT:
                ax.set_ylim(3.5, 0.5)

            ax2 = ax.twiny()
            ax2.set_xticks( ax.get_xticks() )
            ax2.set_xbound(ax.get_xbound())
            x_tickslabels = []
            for i,t in enumerate(x_ticks):
                if i%2==0:
                    x_tickslabels.append(t)
                else:
                    x_tickslabels.append("")
            ax2.set_xticklabels(x_tickslabels, rotation=90)
            ax2.set_xlabel("Time (s)", rotation=0)
            ax2.format_coord = lambda x, y: 't={:g} s'.format(x)
                
            # TIMEOUTS #
            rec = ax.barh( ['TO'], [signal_width_text], left=[0.0], height=1.0, color=(0,0,0,0))
            for sig in g_to_signals:
                # if int(sig.stamp)==77:
                #     continue
                rec = ax.barh( ['TO'], [signal_width_text], left=[sig.stamp-signal_width_text/2], height=0.5, color=(0,0,0,0))
                ax.bar_label(rec, labels=["TO     "], label_type='center', rotation=90, zorder=signal_text_zorder)
                arrow = mpatches.FancyArrowPatch((sig.stamp, 0.25), (sig.stamp , 0.5), color="black", arrowstyle=signal_style, zorder=signal_arrow_zorder)
                ax.add_patch(arrow)

            # ROBOT ACTIVITES #
            for act in g_r_activities:
                if act.dur() < MIN_DURATION:
                    continue

                text = act.name
                color = 'lightskyblue'
                text_color = 'black'
                if act.name in g_r_activities_names:
                    text = g_r_activities_names[act.name][0]
                    color = g_r_activities_names[act.name][1]
                    text_color = g_r_activities_names[act.name][2]
                
                rec = ax.barh( ['R'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color, zorder=activities_zorder)
                ax.bar_label(rec, labels=[text], label_type='center', rotation=90, color=text_color)
                

            # SIGNALS #
            # R Signals #
            for sig in g_r_signals:
                rec = ax.barh( ['Signals'], [signal_width_text], left=[sig.stamp-signal_width_text/2], height=1.0, color=(0,0,0,0))
                new_s_name = sig.name
                if sig.name[:2]=="NS":
                    if sig.name=="NS":
                        new_s_name = "START"
                    elif sig.name=="NS_IDLE":
                        new_s_name = "START_IDLE"
                ax.bar_label(rec, labels=[new_s_name], label_type='center', rotation=90, color=sig.color_text, zorder=signal_text_zorder)
                arrow = mpatches.FancyArrowPatch((sig.stamp, 1.5), (sig.stamp , 2.5), color=sig.color_arrow, arrowstyle=signal_style, zorder=signal_arrow_zorder)
                ax.add_patch(arrow)
                # bar step separations
                if sig.name[:2]=="NS":
                    line = mpatches.FancyArrowPatch((sig.stamp, -1.0), (sig.stamp , 3.5), color="black", arrowstyle="Simple, head_width=0.1, head_length=0.1, tail_width=2", zorder=ns_lines_zorder)
                    ax.add_patch(line)
            # H Signals #
            for sig in g_h_signals:
                rec = ax.barh( ['Signals'], [signal_width_text], left=[sig.stamp-signal_width_text/2], height=1.0, color=(0,0,0,0))
                ax.bar_label(rec, labels=[sig.name], label_type='center', rotation=90, color=sig.color_text, zorder=signal_text_zorder)
                arrow = mpatches.FancyArrowPatch((sig.stamp, 2.5), (sig.stamp , 1.5), color=sig.color_arrow, arrowstyle=signal_style, zorder=signal_arrow_zorder)
                ax.add_patch(arrow)
            
                

            # HUMAN ACTIVITES #
            for act in g_h_activities:
                if act.dur() < MIN_DURATION:
                    continue

                text = act.name
                color = 'wheat'
                text_color = 'black'
                if act.name in g_h_activities_names:
                    text = g_h_activities_names[act.name][0]
                    color = g_h_activities_names[act.name][1]
                    text_color = g_h_activities_names[act.name][2]
                if act.h_is_best:
                    text+="*"

                rec = ax.barh( ['H'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color, zorder=activities_zorder)
                ax.bar_label(rec, labels=[text], label_type='center', rotation=90, color=text_color)

            
            # OVER bar
            over_event = g_events[-1]
            line = mpatches.FancyArrowPatch((over_event.stamp, -1.0), (over_event.stamp , 3.5), color="black", arrowstyle="Simple, head_width=0.1, head_length=0.1, tail_width=2", zorder=ns_lines_zorder)
            ax.add_patch(line)


            plt.tight_layout()

            plt.show()