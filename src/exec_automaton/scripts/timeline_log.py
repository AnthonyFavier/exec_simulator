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

from sim_msgs.msg import EventLog


path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)

## LOGGER ##
logging.config.fileConfig(path + 'log.conf')

class Event:
    def __init__(self, name: str, stamp: float) -> None:
        self.name = name
        self.stamp = stamp

class Activity:
    def __init__(self, name, t_s, t_e) -> None:
        self.name = name
        self.t_s = t_s
        self.t_e = t_e

# g_r_events = [] # List[(name, timestamp)]
g_r_events = [] # List[Event(name, timestamp)]
g_r_activites = [] # List[(name, start, end)]
g_h_events = [] # List[(name, timestamp)]
g_h_activites = [] # List[(name, start, end)]
g_g_activites = [] # List[(name, start, end)]

def log_r_cb(msg: EventLog):
    global g_r_events
    print(msg)
    # g_r_events.append( [msg.name, msg.timestamp] )
    g_r_events.append( Event(msg.name, msg.timestamp) )

def log_h_cb(msg):
    global g_h_events
    print(msg)
    # g_h_events.append( [msg.name, msg.timestamp] )
    g_h_events.append( Event(msg.name, msg.timestamp) )

def reset_times():
    global g_r_events, g_h_events

    g_r_events.sort(key=getStamp)
    g_h_events.sort(key=getStamp)

    if len(g_r_events)>0 and len(g_h_events)>0:
        min_time = min( g_r_events[0].stamp, g_h_events[0].stamp )
    elif len(g_r_events)>0:
        min_time = g_r_events[0].stamp
    elif len(g_h_events)>0:
        min_time = g_h_events[0].stamp
    else:
        raise Exception("Both events empty...")


    for e in g_r_events+g_h_events:
        e.stamp -= min_time

def takeSecond(elem):
    return elem[1]

def getStamp(event):
    return event.stamp

def r_convert_event_to_activities():
    global g_r_events, g_r_activites
    
    # Start Step (When sending VHA)
    i = 0
    e = g_r_events[i]
    while True:

        e=e

        # Waiting for Human Choice
        if e.name == "S_WAIT_HC":
            t_swhc = e.stamp
            while e.name!="E_WAIT_HC" and e.name!="TIMEOUT":
                i+=1
                e = g_r_events[i]
            t_ewhc = e.stamp
            # g_r_activites.append( ("WAIT_HC", t_swhc, t_ewhc) )
            g_r_activites.append( Activity("WAIT_HC", t_swhc, t_ewhc) )

        # ID Phase
        if e.name == "S_ID":
            t_sid = e.stamp
            while e.name!="E_ID":
                i+=1
                e = g_r_events[i]
            t_eid = e.stamp
            g_r_activites.append( Activity("ID", t_sid, t_eid) )

        # Robot PASS
        if e.name == "S_PASS":
            t_spass = e.stamp
            while e.name != "S_ASSESS":
                i+=1
                e = g_r_events[i]
            t_epass = e.stamp
            g_r_activites.append( Activity("PASS", t_spass, t_epass) )

        # Assessing Phase
        if e.name=="S_ASSESS":
            t_sass = e.stamp
            while e.name!="E_ASSESS":
                i+=1
                e = g_r_events[i]
            t_eass = e.stamp
            g_r_activites.append( Activity("ASSESS", t_sass, t_eass) )

        # Go Idle Mode
        if e.name == "S_GO_IDLE":
            t_sgoidle = e.stamp
            while e.name!="E_GO_IDLE":
                i+=1
                e = g_r_events[i]
            t_egoidle = e.stamp
            g_r_activites.append( Activity("GO_IDLE", t_sgoidle, t_egoidle) )
            t_sidle = t_egoidle
            while e.name!="S_END_IDLE":
                i+=1
                e = g_r_events[i]
            t_eidle = e.stamp
            g_r_activites.append( Activity("IDLE", t_sidle, t_eidle) )
            t_sendidle = t_eidle
            while e.name != "E_END_IDLE":
                i+=1
                e = g_r_events[i]
            t_eendidle = e.stamp
            g_r_activites.append( Activity("END_IDLE", t_sendidle, t_eendidle) )

        # Robot action
        if e.name[:4] == "S_RA":
            t_sra = e.stamp
            while e.name[:4]!="E_RA":
                i+=1
                e = g_r_events[i]
            t_era = e.stamp
            g_r_activites.append( Activity(e.name[5:], t_sra, t_era) )

        # Robot action
        if e.name == "E_WAIT_END_HA":
            t_e = e.stamp
            j = i-1
            while g_r_events[j].name[:4]!="E_RA" and g_r_events[j].name!="S_PASS" and g_r_events[j].name!="S_GO_IDLE":
                j-=1
            if not g_r_events[j].name in ["S_PASS", "S_GO_IDLE"]:
                t_s = g_r_events[j].stamp
                if t_e - t_s > 0.2:
                    g_r_activites.append( Activity("WAIT_END_HA", t_s, t_e) )

        
        if e.name == "OVER":
            break
        i+=1
        e = g_r_events[i]

def h_convert_event_to_activities():
    global g_h_events, g_h_activites
    
    # Start Step (When sending VHA)
    i = 0
    e = g_h_events[i]
    while True:

        # Start Delay or WAIT
        if e.name == "NewStep":
            t_ssd = e.stamp
            while e.name[:4] != "S_HA" and e.name != "TIMEOUT" and e.name != "S_PASS":
                i+=1
                e = g_h_events[i]
            t_esd = e.stamp
            if e.name == "TIMEOUT":
                name = "WAIT"
            else:
                name = "StartDelay"
            g_h_activites.append( Activity(name, t_ssd, t_esd) )

        # PASS
        if e.name == "S_PASS":
            t_spass = e.stamp
            while e.name[:4] != "S_HA" and e.name != "NewStep":
                i+=1
                e = g_h_events[i]
            t_epass = e.stamp
            if e.name == "NewStep":
                i-=1
            g_h_activites.append( Activity("PASS", t_spass, t_epass) )

        # Compliant Delay
        if e.name == "TIMEOUT":
            t_scd = e.stamp
            while e.name[:4] != "S_HA" and e.name != "NewStep":
                i+=1
                e = g_h_events[i]
            t_ecd = e.stamp
            if e.name == "NewStep":
                i-=1
                if g_h_activites[-1].name == "WAIT":
                    g_h_activites[-1].t_e = t_ecd
                else:
                    g_h_activites.append( Activity("WAIT", t_scd, t_ecd) )
            else:
                g_h_activites.append( Activity("CompliantDelay", t_scd, t_ecd) )

        # Human action
        if e.name[:4] == "S_HA":
            t_sha = e.stamp
            while e.name[:4]!="E_HA":
                i+=1
                e = g_h_events[i]
            t_eha = e.stamp
            g_h_activites.append( Activity(e.name[5:], t_sha, t_eha) )

        # OVER
        if e.name == "OVER":
            break
        i+=1
        e = g_h_events[i]

def g_events_to_activities():
    global g_g_activites
    # get NewStep, TIMEOUT, and OVER events
    i = 0
    e = g_r_events[i]

    durr_event = 0.5

    while True:
        if e.name == "NewStep":
            g_g_activites.append( Activity("NewStep", e.stamp, e.stamp+durr_event) )

        if e.name == "TIMEOUT":
            g_g_activites.append( Activity("TIMEOUT", e.stamp, e.stamp+durr_event) )

        if e.name == "OVER":
            g_g_activites.append( Activity("OVER", e.stamp, e.stamp+durr_event) )
            break

        i+=1
        e = g_r_events[i]


def show_events(events):
    for e in events:
        print(f"{e.name} - {e.stamp:.2f}")

##########
## MAIN ##
##########

if __name__ == "__main__":
    sys.setrecursionlimit(100000)

    # sys.argv.append("load")

    if len(sys.argv)<2:
        raise Exception("Missing argument... ['load', 'dump']")
    if sys.argv[1] == "load":
        dumping = False
    if sys.argv[1] == "dump":
        dumping = True

    # ROS Startup
    rospy.init_node('timeline_log')

    # Subscribers
    r_log_sub = rospy.Subscriber('/r_event_log', EventLog, log_r_cb)
    h_log_sub = rospy.Subscriber('/h_event_log', EventLog, log_h_cb)

    if dumping:
        print("Listening events...")
        input()

        # Dumping
        events = (g_r_events, g_h_events)
        dill.dump(events, open("/home/afavier/exec_simulator_ws/events.p", "wb"))
        print("events dumped")
    else:
        # Loading
        events = dill.load(open("/home/afavier/exec_simulator_ws/events.p", "rb"))
        g_r_events = events[0]
        g_h_events = events[1]
        print("events loaded")

    reset_times()

    print("\nROBOT EVENTS:")
    show_events(g_r_events)
    print("\nHUMAN EVENTS:")
    show_events(g_h_events)

    r_convert_event_to_activities()
    h_convert_event_to_activities()
    g_events_to_activities()

    fig, ax = plt.subplots(figsize=(18, 5))
    ax.invert_yaxis()
    
    ax.set_axisbelow(True)
    ax.grid(color='lightgrey', linestyle='dashed', axis='x')
    tick_spacing = 2
    # ax.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
    max_x = math.ceil(g_g_activites[-1].t_e)
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



    # ROBOT #
    for act in g_r_activites:
        color = 'lightskyblue'
        if act.name=="WAIT_HC":
            color = "yellow"
        if act.name in ["ID", "ASSESS"]:
            color = "green"
        if act.name in ["GO_IDLE", "END_IDLE"]:
            color = "silver"
        if act.name=="IDLE":
            color = "lightgrey"
        if act.name=="WAIT_END_HA":
            color = "yellow"
        
        rec = ax.barh( ['R'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color)
        ax.bar_label(rec, labels=[act.name], label_type='center', rotation=90)
        
    # GLOBAL #
    for act in g_g_activites:
        color = 'orange'
        rec = ax.barh( ['Events'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color)
        ax.bar_label(rec, labels=[act.name], label_type='center', rotation=90)

    # HUMAN #
    for act in g_h_activites:
        color = 'wheat'
        if act.name in ["StartDelay", "CompliantDelay"]:
            color = "yellow"
        if act.name == "WAIT":
            color = "darkgrey"
        if act.name == "PASS":
            color = "lightgrey"

        rec = ax.barh( ['H'], [act.t_e-act.t_s], left=[act.t_s], height=1.0, color=color)
        ax.bar_label(rec, labels=[act.name], label_type='center', rotation=90)


    plt.tight_layout()

    plt.show()