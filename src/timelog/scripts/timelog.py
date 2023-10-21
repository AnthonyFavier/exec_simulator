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
g_r_activities_names = { 
    # act_name :            [display_name,                  shape_color,            text_color]
    "rf_wait_h":            ["Wait H",                      "yellow",               "black"],
    "wait_hc":              ["Wait H",                      "yellow",               "black"],
    "plan_mvt":             ["Plan Mvt",                    "silver",               "black"],
    "idle_wait_h":          ["Passive\nWait HA",            "silver",               "black"],
    "grns":                 ["GRNS",                        "silver",               "black"],
    "id":                   ["ID Phase",                    "lightgreen",           "black"],
    "sa":                   ["SA",                          "lightgreen",           "black"],
    "wait_end_step":        ["Wait End Step",               "forestgreen",          "white"],
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
                    action_name = e.name[:j_split] + "\n" + e.name[j_split:]

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
                    action_name = e.name[:j_split] + "\n" + e.name[j_split:]
                    
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
            g_r_activities.append( Activity("GRNS", t2, t1) )

            if e.name == "OVER":
                break
def h_extract_activities():
    global g_events, g_h_activities

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
                    action_name = e.name[:j_split] + "\n" + e.name[j_split:]
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

            if e.name[:4]=="S_HA":
                last_act_name = "wait_ns"
                t3 = e.stamp
                j_split = e.name.find("(")
                action_name = e.name[:j_split] + "\n" + e.name[j_split:]
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
        print(f"{a.name} - {a.t_s:.2f} > {a.t_e:.2f}")

##########
## MAIN ##
##########

if __name__ == "__main__":
    sys.setrecursionlimit(100000)

    # sys.argv.append("load")
    # sys.argv.append("events.p")
    record = sys.argv[1] == "record"

    path = "/home/afavier/new_exec_sim_ws/events/"

    # ROS Startup
    rospy.init_node('timelog')

    # Subscribers
    log_sub = rospy.Subscriber('/event_log', EventLog, log_cb)
    log_r_sgl_sub = rospy.Subscriber('/robot_visual_signals', Signal, r_sgl_cb)
    log_h_sgl_sub = rospy.Subscriber('/human_visual_signals', Signal, h_sgl_cb)

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
            dill.dump((g_events, g_r_signals, g_h_signals, g_to_signals), open(path + str_date + "_events.p", "wb"))
            print("events dumped")

            # Clearing
            g_events.clear()
            g_r_signals.clear()
            g_h_signals.clear()
            g_to_signals.clear()

    else:
        file = sys.argv[2]

        # Loading
        (g_events, g_r_signals, g_h_signals, g_to_signals) = dill.load(open(path + file, "rb"))
        print("events loaded")

        # TREAT EVENTS
        reset_times()
        print("\nEVENTS:")
        show_events(g_events)
        
        # TREAT ACTIVITIES
        r_extract_activities()
        print("\nROBOT ACTIVITIES:")
        show_activities(g_r_activities)
        h_extract_activities()
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
        horizontal_space = 0.2
        max_x = g_events[-1].stamp
        ax.set_xlim((-horizontal_space,max_x+horizontal_space))
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