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


path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)

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

    g_events.sort(key=getStamp)

    if len(g_events)!=0:
        min_time = g_events[0].stamp
    else:
        raise Exception("Events empty...")

    for x in g_events + g_r_signals + g_h_signals + g_to_signals:
        x.stamp -= min_time

def getStamp(event):
    return event.stamp

# Events Callback
def log_cb(msg):
    global g_events
    print(msg)
    # g_events.append( Event(msg.name, msg.timestamp) )
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

    print(sgl)

    if sgl.type == Signal.TO:
        # t = rospy.get_time()
        t = time.time()
        TO_name = "TIMEOUT" 
        g_to_signals.append( LogSignal(TO_name, t, sgl.type, None, None) )
        g_events.append( Event("SGL_"+TO_name, t) )

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
def h_sgl_cb(sgl: Signal):
    global g_h_signals

    print(sgl)
    
    if sgl.type in g_h_signals_names:
        # t = rospy.get_time()
        t = time.time()
        g_h_signals.append( LogSignal(g_h_signals_names[sgl.type][0], t, sgl.type, g_h_signals_names[sgl.type][1], g_h_signals_names[sgl.type][2]) )
        g_events.append( Event("SGL_"+g_h_signals_names[sgl.type][0], t) )

# Activities Extraction
g_r_activities_names = { # act_name : [display_name, shape_color, text_color]
    "wait_hc":          ["Wait H",              "yellow",       "black"],
    "idle_wait_h":      ["Passive\nWait HA",    "silver",       "black"],
    "wait_end_ha":      ["Wait E_HA",           "forestgreen",  "white"],
    "id":               ["ID Phase",            "lightgreen",   "black"],
    "sa":               ["SA",                  "lightgreen",   "black"],
    "plan_mvt":         ["Plan Mvt",            "silver",       "black"],
    "grns":             ["GRNS",                "silver",       "black"],
}
g_h_activities_names = { # act_name : [display_name, shape_color, text_color]
    "idle_wait_ns":         ["Wait NS",             "forestgreen",   "white"],
    "idle_pass_wait_ns":    ["Wait NS",             "forestgreen",   "white"],
    "idle_wait_wait_ns":    ["Wait NS",             "forestgreen",   "white"],
    "idle_wait_compliant":  ["Passive\nNo Signal",  "lightgrey",     "black"],
    "idle_pass_compliant":  ["Passive\nPASS",       "lightgrey",     "black"],
    "start_delay":          ["Decision",            "yellow",        "black"],
}
def r_extract_activities():
    global g_events, g_r_activities

    i = 0
    e = g_events[i]
    while e.name[:6]!="SGL_NS":
        i+=1
        e = g_events[i]
    while True:

        if e.name == "SGL_NS":
            
            t1 = e.stamp
            while e.name!="R_E_WAIT_HC":
                i+=1
                e = g_events[i]
            tsplan = e.stamp
            g_r_activities.append( Activity("wait_hc", t1, tsplan) )

            while e.name[:4]!="S_RA" and e.name!="SGL_R_PASS" and e.name!="R_E_ID":
                i+=1
                e = g_events[i]
                
            if e.name=="R_E_ID":
                g_r_activities.append( Activity("id", tsplan, e.stamp) )
                tsplan = e.stamp
                while e.name[:4]!="S_RA" and e.name!="SGL_R_PASS":
                    i+=1
                    e = g_events[i]

            if e.name=="SGL_R_PASS":
                tsweha = e.stamp
                g_r_activities.append( Activity("plan_mvt", tsplan, tsweha) )

            if e.name[:4]=="S_RA":
                tsra = e.stamp
                act_name = e.name[5:]
                j_split = act_name.find("(")
                act_name = act_name[:j_split] + "\n" + act_name[j_split:]
                while e.name!="SGL_S_RA":
                    i+=1
                    e = g_events[i]
                tssra = e.stamp
                g_r_activities.append( Activity("plan_mvt", tsra, tssra) )

                while e.name!="SGL_E_RA":
                    i+=1
                    e = g_events[i]
                tsweha = e.stamp
                g_r_activities.append( Activity(act_name, tssra, tsweha) )

            

        elif e.name=="SGL_NS_IDLE":
            t1 = e.stamp
            while e.name!="R_E_WAIT_HSA":
                i+=1
                e = g_events[i]
            tsweha = e.stamp
            g_r_activities.append( Activity("idle_wait_h", t1, tsweha) )

        while e.name!="R_S_ASSESS":
            i+=1
            e = g_events[i]
        teweha = e.stamp
        g_r_activities.append( Activity("wait_end_ha", tsweha, teweha) )

        while e.name!="R_E_ASSESS":
            i+=1
            e = g_events[i]
        teass = e.stamp
        g_r_activities.append( Activity("sa", teweha, teass) )

        while e.name[:6]!="SGL_NS" and e.name!="OVER":
            i+=1
            e = g_events[i]
        tf = e.stamp
        g_r_activities.append( Activity("grns", teass, tf) )
        if e.name == "OVER":
            break
def h_extract_activities():
    global g_events, g_h_activities
    
    i = 0
    e = g_events[i]
    while e.name[:6]!="SGL_NS":
        i+=1
        e = g_events[i]
    while True:

        t1 = t2 = t3 = t4 = t5 = t6 = t7 = None

        if e.name=="SGL_NS":
            t1 = e.stamp

            while e.name!="SGL_TIMEOUT" and e.name[:4]!="S_HA" and e.name!="SGL_H_PASS":
                i+=1
                e = g_events[i]
            
            if e.name=="SGL_TIMEOUT":
                t2 = e.stamp
                while e.name!="OVER" and e.name!="SGL_NS" and e.name!="SGL_NS_IDLE" and e.name[:4]!="S_HA":
                    i+=1
                    e = g_events[i]

            elif e.name=="SGL_H_PASS":
                t5 = e.stamp
                g_h_activities.append( Activity("start_delay", t1, t5) )
                while e.name!="OVER" and e.name!="SGL_NS" and e.name!="SGL_NS_IDLE" and e.name[:4]!="S_HA":
                    i+=1
                    e = g_events[i]

        elif e.name=="SGL_NS_IDLE":
            t6 = e.stamp
            while e.name[:4]!="S_HA":
                    i+=1
                    e = g_events[i]

        if e.name[:4]=="S_HA":
            t3 = e.stamp
            if t6!=None:
                g_h_activities.append( Activity("start_delay", t6, t3))
            elif t2==None and t5==None:
                g_h_activities.append( Activity("start_delay", t1, t3))
            elif t2!=None:
                g_h_activities.append( Activity("idle_wait_compliant", t1, t3))
            elif t5!=None:
                g_h_activities.append( Activity("idle_pass_compliant", t5, t3))
            act_name = e.name[5:]
            j_split = act_name.find("(")
            act_name = act_name[:j_split] + "\n" + act_name[j_split:]
            while e.name!="SGL_E_HA":
                i+=1
                e = g_events[i]
            t4 = e.stamp
            g_h_activities.append( Activity(act_name, t3, t4) )

            while e.name!="OVER" and e.name!="SGL_NS" and e.name!="SGL_NS_IDLE":
                i+=1
                e = g_events[i]

        if e.name=="OVER" or e.name=="SGL_NS" or e.name=="SGL_NS_IDLE":
            t7 = e.stamp
            if t4!=None:
                g_h_activities.append( Activity("idle_wait_ns", t4, t7) )
            elif t2!=None:
                g_h_activities.append( Activity("idle_wait_wait_ns", t1, t7) )
            elif t5!=None:
                g_h_activities.append( Activity("idle_pass_wait_ns", t5, t7) )

        if e.name=="OVER":
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
        print(f"{a.name} - {a.t_s:.2f} > {a.t_e:.2f}")

##########
## MAIN ##
##########

if __name__ == "__main__":
    sys.setrecursionlimit(100000)

    # sys.argv.append("load")
    if len(sys.argv)<2:
        raise Exception("Missing argument... ['load', 'record']")
    record = sys.argv[1] == "record"

    # ROS Startup
    rospy.init_node('timelog')

    # Subscribers
    log_sub = rospy.Subscriber('/event_log', EventLog, log_cb)
    log_r_sgl_sub = rospy.Subscriber('/robot_visual_signals', Signal, r_sgl_cb)
    log_h_sgl_sub = rospy.Subscriber('/human_visual_signals', Signal, h_sgl_cb)

    s_t = time.time()

    if record:
        print("Listening events... (type 'q' and return to abort)")
        t = input()
        if t=="q":
            print("Record aborted")
            exit(1)

        # Dumping
        dill.dump((g_events, g_r_signals, g_h_signals, g_to_signals), open("/home/afavier/new_exec_sim_ws/events.p", "wb"))
        print("events dumped")
    else:
        # Loading
        (g_events, g_r_signals, g_h_signals, g_to_signals) = dill.load(open("/home/afavier/new_exec_sim_ws/events.p", "rb"))
        print("events loaded")

    e_t = time.time()

    print(s_t)
    print(e_t)
    print(f"Elapsed t : {e_t-s_t}")

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
    print("\nR SIGNALS:")
    show_signals(g_r_signals)
    print("\nH SIGNALS:")
    show_signals(g_h_signals)
    print("\nTO SIGNALS:")
    show_signals(g_to_signals)

    MIN_DURATION = 0.15
    activities_zorder = 1
    ns_lines_zorder = 2
    signal_arrow_zorder = 3
    signal_text_zorder = 10
    signal_width_text = 1.5
    signal_style="Simple, head_width=8, head_length=4, tail_width=4"

########################
    WITH_TIMEOUT = True
########################


    plt.rcParams.update({'font.size': 13})

    fig, ax = plt.subplots(figsize=(18, 5))
    ax.invert_yaxis()
    
    ax.set_axisbelow(True)
    ax.grid(color='lightgrey', linestyle='dashed', axis='x')
    tick_spacing = 1
    max_x = math.ceil(g_events[-1].stamp)
    ax.set_xlim((-0.2,max_x))
    x_ticks = np.arange(0, max_x, tick_spacing )
    ax.set_xticks(x_ticks)
    ax.set_xticklabels(["" for x in x_ticks], rotation=90)
    ax.set_yticks([0.25, 1.0, 2.0, 3.0])
    ax.set_yticklabels(['TO', 'R', 'Sgls', 'H'], rotation=90)
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
        ax.bar_label(rec, labels=[sig.name], label_type='center', rotation=90, color=sig.color_text, zorder=signal_text_zorder)
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

        rec = ax.barh( ['H'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color, zorder=activities_zorder)
        ax.bar_label(rec, labels=[text], label_type='center', rotation=90, color=text_color)

    
    # OVER bar
    over_event = g_events[-1]
    line = mpatches.FancyArrowPatch((over_event.stamp, -1.0), (over_event.stamp , 3.5), color="black", arrowstyle="Simple, head_width=0.1, head_length=0.1, tail_width=2", zorder=ns_lines_zorder)
    ax.add_patch(line)


    plt.tight_layout()

    plt.show()