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

def old_r_extract_activities():
    global g_events, g_r_activities

    # Start Step (When sending VHA)
    i = 0
    e = g_events[i]
    while True:

        e=e

        # Waiting for Human Choice
        if e.name == "R_S_WAIT_HC":
            t_swhc = e.stamp
            while e.name!="R_E_WAIT_HC" and e.name!="R_TIMEOUT":
                i+=1
                e = g_events[i]
            t_ewhc = e.stamp
            # g_r_activities.append( ("WAIT_HC", t_swhc, t_ewhc) )
            g_r_activities.append( Activity("WAIT_HC", t_swhc, t_ewhc) )

        # ID Phase
        if e.name == "R_S_ID":
            t_sid = e.stamp
            while e.name!="R_E_ID":
                i+=1
                e = g_events[i]
            t_eid = e.stamp
            g_r_activities.append( Activity("ID", t_sid, t_eid) )

        # Robot PASS
        if e.name == "R_PASS":
            t_spass = e.stamp
            while e.name != "R_S_ASSESS":
                i+=1
                e = g_events[i]
            t_epass = e.stamp
            g_r_activities.append( Activity("PASS", t_spass, t_epass) )

        # Assessing Phase
        if e.name=="R_S_ASSESS":
            t_sass = e.stamp
            while e.name!="R_E_ASSESS":
                i+=1
                e = g_events[i]
            t_eass = e.stamp
            g_r_activities.append( Activity("ASSESS", t_sass, t_eass) )

        # IDLE
        if e.name == "NS_IDLE":
            t_sidle = e.stamp
            while e.name[:4] != "E_HA":
                i+=1
                e = g_events[i]
            t_eidle = e.stamp
            g_r_activities.append( Activity("IDLE", t_sidle, t_eidle) )

        # Robot action
        if e.name[:4] == "S_RA":
            t_sra = e.stamp
            while e.name[:4]!="E_RA":
                i+=1
                e = g_events[i]
            t_era = e.stamp
            g_r_activities.append( Activity(e.name[5:], t_sra, t_era) )

        # Robot action
        if e.name == "R_E_WAIT_END_HA":
            t_e = e.stamp
            j = i-1
            while g_events[j].name[:4]!="E_RA" and g_events[j].name!="R_PASS" and g_events[j].name!="S_GO_IDLE":
                j-=1
            if not g_events[j].name in ["R_PASS", "S_GO_IDLE"]:
                t_s = g_events[j].stamp
                if t_e - t_s > 0.2:
                    g_r_activities.append( Activity("WAIT_END_HA", t_s, t_e) )

        
        if e.name == "OVER":
            break
        i+=1
        e = g_events[i]

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
            g_r_activities.append( Activity("WAIT_HC", t1, tsplan) )

            while e.name[:4]!="S_RA" and e.name!="R_PASS" and e.name!="R_E_ID":
                i+=1
                e = g_events[i]

            if e.name=="R_E_ID":
                g_r_activities.append( Activity("ID", tsplan, e.stamp) )
                tsplan = e.stamp
                while e.name[:4]!="S_RA" and e.name!="R_PASS":
                    i+=1
                    e = g_events[i]

            if e.name[:4]=="S_RA":
                tsra = e.stamp
                g_r_activities.append( Activity("Plan mvt", tsplan, tsra) )

                while e.name[:4]!="E_RA":
                    i+=1
                    e = g_events[i]
                tsweha = e.stamp
                g_r_activities.append( Activity(e.name[5:], tsra, tsweha) )

            elif e.name=="R_PASS":
                tsweha = e.stamp

        elif e.name=="NS_IDLE":
            t1 = e.stamp
            while e.name[:4]!="S_HA":
                i+=1
                e = g_events[i]
            tsweha = e.stamp
            g_r_activities.append( Activity("IDLE\nWAIT_HA", t1, tsweha) )

            
        while e.name!="R_S_ASSESS":
            i+=1
            e = g_events[i]
        teweha = e.stamp
        g_r_activities.append( Activity("WAIT_E_HA", tsweha, teweha) )

        while e.name!="R_E_ASSESS":
            i+=1
            e = g_events[i]
        teass = e.stamp
        g_r_activities.append( Activity("SA", teweha, teass) )

        while e.name[:2]!="NS" and e.name!="OVER":
            i+=1
            e = g_events[i]
        tf = e.stamp
        g_r_activities.append( Activity("GRNS", teass, tf) )
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
                    g_h_activities.append( Activity("IDLE (wait)\n Wait NS", t1, t3) )
                    i-=1
                    e = g_events[i]
                # 2 #
                elif e.name[:4]=="S_HA":
                    g_h_activities.append( Activity("IDLE (wait)\n Compliant", t1, t3) )

                    while e.name[:4]!="E_HA":
                        i+=1
                        e = g_events[i]
                    t4 = e.stamp
                    g_h_activities.append( Activity(e.name[5:], t3, t4) )

                    while e.name[:2]!="NS" and e.name!="OVER":
                        i+=1
                        e = g_events[i]
                    t5 = e.stamp
                    g_h_activities.append( Activity("IDLE Wait NS", t4, t5) )
                    i-=1
                    e = g_events[i]


            # 3 #
            elif e.name[:4]=="S_HA":
                g_h_activities.append( Activity("StartDelay", t1, t2) )

                while e.name[:4]!="E_HA":
                    i+=1
                    e = g_events[i]
                t3 = e.stamp
                g_h_activities.append( Activity(e.name[5:], t2, t3) )

                while e.name[:2]!="NS" and e.name!="OVER":
                    i+=1
                    e = g_events[i]
                t4 = e.stamp
                g_h_activities.append( Activity("IDLE Wait NS", t3, t4) )
                i-=1
                e = g_events[i]
            
            # 4 # 5 #
            elif e.name=="H_PASS":
                g_h_activities.append( Activity("StartDelay", t1, t2) )

                while e.name[:2]!="NS" and e.name!="OVER" and e.name[:4]!="S_HA":
                    i+=1
                    e = g_events[i]
                t3 = e.stamp

                # 5 #
                if e.name[:2]=="NS" or e.name=="OVER":
                    g_h_activities.append( Activity("IDLE (pass)\nWait NS", t2, t3) )
                    i-=1
                    e = g_events[i]
                
                # 4 #
                elif e.name[:4]=="S_HA":
                    g_h_activities.append( Activity("IDLE (pass)\nCompliant", t2, t3) )

                    while e.name[:4]!="E_HA":
                        i+=1
                        e = g_events[i]
                    t4 = e.stamp
                    g_h_activities.append( Activity(e.name[5:], t3, t4) )

                    while e.name[:2]!="NS" and e.name!="OVER":
                        i+=1
                        e = g_events[i]
                    t5 = e.stamp
                    g_h_activities.append( Activity("IDLE Wait NS", t4, t5) )
                    i-=1
                    e = g_events[i]

        # 6 #
        elif e.name == "NS_IDLE":
            t1 = e.stamp
            while e.name[:4]!="S_HA":
                i+=1
                e = g_events[i]
            t2 = e.stamp
            g_h_activities.append( Activity("StartDelay", t1, t2) )

            while e.name[:4]!="E_HA":
                i+=1
                e = g_events[i]
            t3 = e.stamp
            g_h_activities.append( Activity(e.name[5:], t2, t3) )

            while e.name[:2]!="NS" and e.name!="OVER":
                i+=1
                e = g_events[i]
            t4 = e.stamp
            g_h_activities.append( Activity("IDLE Wait NS", t3, t4) )
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

        if e.name[:4] == "S_RA":
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
        if e.name[:4] == "S_HA":
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

    if dumping:
        print("Listening events...")
        input()

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

    fig, ax = plt.subplots(figsize=(18, 5))
    ax.invert_yaxis()
    
    ax.set_axisbelow(True)
    ax.grid(color='lightgrey', linestyle='dashed', axis='x')
    tick_spacing = 2
    # ax.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
    max_x = math.ceil(g_events[-1].stamp)
    ax.set_xlim((0,max_x))
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

    # ROBOT #
    for act in g_r_activities:
        if act.dur() < MIN_DURATION:
            continue

        color = 'lightskyblue'
        text_color = 'black'
        if act.name=="WAIT_HC":
            color = "yellow"
        if act.name in ["ID", "SA"]:
            color = "lightgreen"
        if act.name in ["GO_IDLE", "END_IDLE"]:
            color = "silver"
        if act.name=="IDLE":
            color = "lightgrey"
        if act.name=="PASS":
            color = "lightgrey"
        if act.name=="WAIT_E_HA":
            color = "forestgreen"
            text_color = "white"
        if act.name=="IDLE\nWAIT_HA":
            color = "yellow"
        if act.name=="Plan mvt":
            color = "silver"
        if act.name=="GRNS":
            color = "silver"
        
        rec = ax.barh( ['R'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color, zorder=activities_zorder)
        ax.bar_label(rec, labels=[act.name], label_type='center', rotation=90, color=text_color)
        

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
        

    # HUMAN #
    for act in g_h_activities:
        if act.dur() < MIN_DURATION:
            continue

        color = 'wheat'
        text_color = 'black'
        if act.name in ["StartDelay"]:
            color = "yellow"
        if act.name in ["IDLE (pass)\nCompliant", "IDLE (wait)\n Compliant"]:
            color = "lightgrey"
        if act.name in ["IDLE Wait NS", "IDLE (pass)\nWait NS", "IDLE (wait)\n Wait NS"]:
            color = "forestgreen"
            text_color = "white"

        rec = ax.barh( ['H'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color, zorder=activities_zorder)
        ax.bar_label(rec, labels=[act.name], label_type='center', rotation=90, color=text_color)

    ax.format_coord = lambda x, y: 't={:g} s'.format(x)


    plt.tight_layout()

    plt.show()