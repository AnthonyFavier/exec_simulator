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
import importlib
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

import matplotlib.patches as mpatches

from sim_msgs.msg import EventLog


path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')

# Event(name: str, stamp: float)
class Event:
    def __init__(self, name: str, stamp: float) -> None:
        self.name = name
        self.stamp = stamp

# Activity(name: str, t_s: float, t_e: float)
class Activity:
    def __init__(self, name, t_s, t_e) -> None:
        self.name = name
        self.t_s = t_s
        self.t_e = t_e

    def dur(self):
        return self.t_e - self.t_s

g_events = [] #type: List[Event]

g_r_activities = [] #type: List[Activity]
g_h_activities = [] #type: List[Activity]

g_r_signals = [] #type: List[Event]
g_h_signals = [] #type: List[Event]
g_to_signals = [] #type: List[Event]

def log_cb(msg):
    global g_events
    print(msg)
    g_events.append( Event(msg.name, msg.timestamp) )

def reset_times():
    global g_events

    g_events.sort(key=getStamp)

    if len(g_events)!=0:
        min_time = g_events[0].stamp
    else:
        raise Exception("Events empty...")


    for e in g_events:
        e.stamp -= min_time

def takeSecond(elem):
    return elem[1]

def getStamp(event):
    return event.stamp


g_r_activities_names     = {
    "wait_hc":          ["Wait H",              "yellow",       "black"],
    "idle_wait_s_ha":   ["Wait H",              "yellow",       "black"],
    "wait_e_ha":        ["Wait E_HA",  "forestgreen",  "white"],
    "id":               ["ID Phase",            "lightgreen",   "black"],
    "sa":               ["SA",                  "lightgreen",   "black"],
    "plan_mvt":         ["Plan Mvt",            "silver",       "black"],
    "grns":             ["GRNS",                "silver",       "black"],
}
g_h_activities_names = {
    "idle_wait_ns":         ["Wait NS",         "forestgreen",   "white"],
    "idle_pass_wait_ns":    ["Wait NS",         "forestgreen",   "white"],
    "idle_wait_wait_ns":    ["Wait NS",         "forestgreen",   "white"],
    "idle_wait_compliant":  ["Compliant\nIDLE", "lightgrey",     "black"],
    "idle_pass_compliant":  ["Compliant\nIDLE", "lightgrey",     "black"],
    "start_delay":          ["Start Delay",     "yellow",        "black"],
}
def r_extract_activities():
    global g_events, g_r_activities

    # Start Step (When sending VHA)
    i = 0
    e = g_events[i]
    while True:

        if e.name == "NS":
            t1 = e.stamp
            # while e.name!="H_PASS" and e.name!="R_TIMEOUT" and e.name[:4]!="S_HA":
            while e.name!="R_E_WAIT_HC":
                i+=1
                e = g_events[i]
            tsplan = e.stamp
            g_r_activities.append( Activity("wait_hc", t1, tsplan) )

            while e.name[:4]!="S_RA" and e.name!="R_PASS" and e.name!="R_E_ID":
                i+=1
                e = g_events[i]

            if e.name=="R_E_ID":
                g_r_activities.append( Activity("id", tsplan, e.stamp) )
                tsplan = e.stamp
                while e.name[:4]!="S_RA" and e.name!="R_PASS":
                    i+=1
                    e = g_events[i]

            if e.name[:4]=="S_RA":
                tsra = e.stamp
                g_r_activities.append( Activity("plan_mvt", tsplan, tsra) )

                while e.name[:4]!="E_RA":
                    i+=1
                    e = g_events[i]
                tsweha = e.stamp
                g_r_activities.append( Activity(e.name[5:], tsra, tsweha) )

            elif e.name=="R_PASS":
                tsweha = e.stamp

        elif e.name=="NS_IDLE":
            t1 = e.stamp
            while e.name!="SGL_S_HA":
                i+=1
                e = g_events[i]
            tsweha = e.stamp
            g_r_activities.append( Activity("idle_wait_s_ha", t1, tsweha) )

            
        while e.name!="R_S_ASSESS":
            i+=1
            e = g_events[i]
        teweha = e.stamp
        g_r_activities.append( Activity("wait_e_ha", tsweha, teweha) )

        while e.name!="R_E_ASSESS":
            i+=1
            e = g_events[i]
        teass = e.stamp
        g_r_activities.append( Activity("sa", teweha, teass) )

        while e.name[:2]!="NS" and e.name!="OVER":
            i+=1
            e = g_events[i]
        tf = e.stamp
        g_r_activities.append( Activity("grns", teass, tf) )
        if e.name[:2]=="NS":
            i-=1
            e = g_events[i]

        
        if e.name == "OVER":
            break
        i+=1
        e = g_events[i]
def h_extract_activities():
    global g_events, g_h_activities
    
    
    # Start Step (When sending VHA)
    i = 0
    e = g_events[i]
    while True:

        # 1 # 2 # 3 # 4 # 5 #
        if e.name == "NS":
            t1 = e.stamp
            while e.name[:4]!="S_HA" and e.name!="H_PASS" and e.name!="R_TIMEOUT":
                i+=1
                e = g_events[i]
            t2 = e.stamp


            # 1 # 2 #
            if e.name=="R_TIMEOUT":
                while e.name[:4]!="S_HA" and e.name[:2]!="NS" and e.name!="OVER":
                    i+=1
                    e = g_events[i]
                t3 = e.stamp

                # 1 #
                if e.name[:2]=="NS" or e.name=="OVER":
                    g_h_activities.append( Activity("idle_wait_wait_ns", t1, t3) )
                    i-=1
                    e = g_events[i]
                # 2 #
                elif e.name[:4]=="S_HA":
                    g_h_activities.append( Activity("idle_wait_compliant", t1, t3) )

                    while e.name[:4]!="E_HA":
                        i+=1
                        e = g_events[i]
                    t4 = e.stamp
                    g_h_activities.append( Activity(e.name[5:], t3, t4) )

                    while e.name[:2]!="NS" and e.name!="OVER":
                        i+=1
                        e = g_events[i]
                    t5 = e.stamp
                    g_h_activities.append( Activity("idle_wait_ns", t4, t5) )
                    i-=1
                    e = g_events[i]


            # 3 #
            elif e.name[:4]=="S_HA":
                g_h_activities.append( Activity("start_delay", t1, t2) )

                while e.name[:4]!="E_HA":
                    i+=1
                    e = g_events[i]
                t3 = e.stamp
                g_h_activities.append( Activity(e.name[5:], t2, t3) )

                while e.name[:2]!="NS" and e.name!="OVER":
                    i+=1
                    e = g_events[i]
                t4 = e.stamp
                g_h_activities.append( Activity("idle_wait_ns", t3, t4) )
                i-=1
                e = g_events[i]
            
            # 4 # 5 #
            elif e.name=="H_PASS":
                g_h_activities.append( Activity("start_delay", t1, t2) )

                while e.name[:2]!="NS" and e.name!="OVER" and e.name[:4]!="S_HA":
                    i+=1
                    e = g_events[i]
                t3 = e.stamp

                # 5 #
                if e.name[:2]=="NS" or e.name=="OVER":
                    g_h_activities.append( Activity("idle_pass_wait_ns", t2, t3) )
                    i-=1
                    e = g_events[i]
                
                # 4 #
                elif e.name[:4]=="S_HA":
                    g_h_activities.append( Activity("idle_pass_compliant", t2, t3) )

                    while e.name[:4]!="E_HA":
                        i+=1
                        e = g_events[i]
                    t4 = e.stamp
                    g_h_activities.append( Activity(e.name[5:], t3, t4) )

                    while e.name[:2]!="NS" and e.name!="OVER":
                        i+=1
                        e = g_events[i]
                    t5 = e.stamp
                    g_h_activities.append( Activity("idle_wait_ns", t4, t5) )
                    i-=1
                    e = g_events[i]

        # 6 #
        elif e.name == "NS_IDLE":
            t1 = e.stamp
            while e.name[:4]!="S_HA":
                i+=1
                e = g_events[i]
            t2 = e.stamp
            g_h_activities.append( Activity("start_delay", t1, t2) )

            while e.name[:4]!="E_HA":
                i+=1
                e = g_events[i]
            t3 = e.stamp
            g_h_activities.append( Activity(e.name[5:], t2, t3) )

            while e.name[:2]!="NS" and e.name!="OVER":
                i+=1
                e = g_events[i]
            t4 = e.stamp
            g_h_activities.append( Activity("idle_wait_ns", t3, t4) )
            i-=1
            e = g_events[i]

        # OVER
        elif e.name == "OVER":
            break
        i+=1
        e = g_events[i]

def compute_signals():
    global g_r_signals, g_h_signals

    # In Robot
    i = 0
    e = g_events[i]
    while True:
        if e.name == "NS":
            g_r_signals.append( Event("NS", e.stamp) )

        if e.name == "NS_IDLE":
            g_r_signals.append( Event("NS_IDLE", e.stamp) )

        if e.name == "S_GO_IDLE":
            g_r_signals.append( Event("S_R_IDLE", e.stamp) )

        if e.name == "S_END_IDLE":
            g_r_signals.append( Event("E_R_IDLE", e.stamp,) )

        if e.name == "SGL_S_RA":
            g_r_signals.append( Event("S_RA", e.stamp) )
        
        if e.name == "R_PASS":
            g_r_signals.append( Event("R_PASS", e.stamp) )


        if e.name == "OVER":
            break
        i+=1
        e = g_events[i]

    # In Human
    i = 0
    e = g_events[i]
    while True:
        if e.name == "SGL_S_HA":
            g_h_signals.append( Event("S_HA", e.stamp) )

        if e.name[:4] == "E_HA":
            g_h_signals.append( Event("E_HA", e.stamp) )

        if e.name == "H_PASS":
            g_h_signals.append( Event("H_PASS", e.stamp) )
        
        if e.name == "OVER":
            break
        i+=1
        e = g_events[i]

    # Timeout
    i = 0
    e = g_events[i]
    while True:
        if e.name == "R_TIMEOUT":
            g_to_signals.append( Event("TIMEOUT", e.stamp) )

        if e.name == "OVER":
            break
        i+=1
        e = g_events[i]

def show_events(events):
    for e in events:
        print(f"{e.name} - {e.stamp:.2f}")

def show_activities(activities):
    for a in activities:
        print(f"{a.name} - {a.t_s:.2f} > {a.t_e:.2f}")

##########
## MAIN ##
##########

if __name__ == "__main__":
    sys.setrecursionlimit(100000)

    sys.argv.append("load")

    if len(sys.argv)<2:
        raise Exception("Missing argument... ['load', 'dump']")
    if sys.argv[1] == "load":
        dumping = False
    if sys.argv[1] == "dump":
        dumping = True

    # ROS Startup
    rospy.init_node('timeline_log')

    # Subscribers
    log_sub = rospy.Subscriber('/event_log', EventLog, log_cb)
    # log_r_sgl_sub = rospy.Subscriber('/robot_visual_signals', Signal, r_sgl_cb)
    # log_h_sgl_sub = rospy.Subscriber('/human_visual_signals', Signal, h_sgl_cb)

    if dumping:
        print("Listening events... (type 'q' and return to abort)")
        t = input()
        if t=="q":
            print("Record aborted")
            exit(1)

        # Dumping
        dill.dump(g_events, open("/home/afavier/exec_simulator_ws/events.p", "wb"))
        print("events dumped")
    else:
        # Loading
        g_events = dill.load(open("/home/afavier/exec_simulator_ws/events.p", "rb"))
        print("events loaded")


    # TREAT EVENTS
    reset_times()
    print("\nEVENTS:")
    show_events(g_events)
    
    # TREAT ACTIVITIES
    r_extract_activities()
    h_extract_activities()
    print("\nROBOT ACTIVITIES:")
    show_activities(g_r_activities)
    print("\nHUMAN ACTIVITIES:")
    show_activities(g_h_activities)

    # TREAT SIGNALS
    compute_signals()
    print("\nR SIGNALS:")
    show_events(g_r_signals)
    print("\nH SIGNALS:")
    show_events(g_h_signals)
    print("\nTO SIGNALS:")
    show_events(g_to_signals)

    plt.rcParams.update({'font.size': 13})

    fig, ax = plt.subplots(figsize=(18, 5))
    ax.invert_yaxis()
    
    ax.set_axisbelow(True)
    ax.grid(color='lightgrey', linestyle='dashed', axis='x')
    tick_spacing = 2
    # ax.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
    max_x = math.ceil(g_events[-1].stamp)
    ax.set_xlim((-0.5,max_x))
    x_ticks = np.arange(0, max_x, tick_spacing )
    ax.set_xticks(x_ticks)
    x_tickslabels = []
    for i,t in enumerate(x_ticks):
        if i%2==0:
            x_tickslabels.append(t)
        else:
            x_tickslabels.append("")

    ax.set_xticklabels(x_tickslabels)


    MIN_DURATION = 0.15


    activities_zorder = 1
    ns_lines_zorder = 2
    signal_arrow_zorder = 3
    signal_text_zorder = 10

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
    width_text = 1.5
    color_text_r = "black"
    # color_arrow_r = (0.07, 0.32, 1.0)
    color_arrow_r = "lightskyblue"
    color_text_h = "black"
    # color_arrow_h = (0.85, 0.67, 0)
    color_arrow_h = "wheat"
    style="Simple, head_width=8, head_length=4, tail_width=4"
    # R Signals #
    for sig in g_r_signals:
        rec = ax.barh( ['Signals'], [width_text], left=[sig.stamp-width_text/2], height=1.0, color=(0,0,0,0))
        ax.bar_label(rec, labels=[sig.name], label_type='center', rotation=90, color=color_text_r, zorder=signal_text_zorder)
        arrow = mpatches.FancyArrowPatch((sig.stamp, 0.5), (sig.stamp , 1.5), color=color_arrow_r, arrowstyle=style, zorder=signal_arrow_zorder)
        ax.add_patch(arrow)
        if sig.name[:2]=="NS":
            line = mpatches.FancyArrowPatch((sig.stamp, -0.6), (sig.stamp , 2.6), color="black", arrowstyle="Simple, head_width=0.1, head_length=0.1, tail_width=2", zorder=ns_lines_zorder)
            ax.add_patch(line)
    # H Signals #
    for sig in g_h_signals:
        rec = ax.barh( ['Signals'], [width_text], left=[sig.stamp-width_text/2], height=1.0, color=(0,0,0,0))
        ax.bar_label(rec, labels=[sig.name], label_type='center', rotation=90, color=color_text_h, zorder=signal_text_zorder)
        arrow = mpatches.FancyArrowPatch((sig.stamp, 1.5), (sig.stamp , 0.5), color=color_arrow_h, arrowstyle=style, zorder=signal_arrow_zorder)
        ax.add_patch(arrow)
    # TIMEOUTS #
    for sig in g_to_signals:
        rec = ax.barh( ['Signals'], [width_text], left=[sig.stamp-width_text/2], height=1.0, color=(0,0,0,0))
        ax.bar_label(rec, labels=["TO"], label_type='center', rotation=90, zorder=signal_text_zorder)
        arrow = mpatches.FancyArrowPatch((sig.stamp, 0.9), (sig.stamp , 0.5), color="black", arrowstyle=style, zorder=signal_arrow_zorder)
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

        rec = ax.barh( ['H'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color, zorder=activities_zorder)
        ax.bar_label(rec, labels=[text], label_type='center', rotation=90, color=text_color)

    ax.format_coord = lambda x, y: 't={:g} s'.format(x)


    plt.tight_layout()

    plt.show()