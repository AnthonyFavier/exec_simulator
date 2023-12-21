#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Dict, List, Tuple
from copy import deepcopy
import random
import dill
import sys
from enum import Enum
import logging as lg
import logging.config

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
import matplotlib.ticker as ticker
import math
import time
import os 

import matplotlib.patches as mpatches

from sim_msgs.msg import EventLog
from datetime import datetime

import timelog

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)
import ConcurrentModule as ConM
import CommonModule as CM


g_tee_final = None
g_tee_states = None
g_hfe_final = None
g_hfe_states = None

def loading(file_path):
    (timelog.g_events, timelog.g_r_signals, timelog.g_h_signals, timelog.g_to_signals) = dill.load(open(file_path, "rb"))
    timelog.g_h_activities = []
    timelog.g_r_activities = []
    print("\t events loaded")

    i_s = file_path.find("instru_")+len("instru_")
    i_e = i_s + 3
    h_instru = file_path[i_s:i_e]

    if h_instru=="tee":
        CM.g_FINAL_PSTATES = g_tee_final
        CM.g_PSTATES = g_tee_states
    elif h_instru=="hfe":
        CM.g_FINAL_PSTATES = g_hfe_final
        CM.g_PSTATES = g_hfe_states
    else:
        raise Exception("h_instru unknown...")

def metrics_retrieval():
    timelog.reset_times()
    timelog.r_extract_activities()
    timelog.h_extract_activities()
    # print(timelog.g_events[0].name)

    metrics = timelog.extract_metrics()
    print("\t metrics extracted")

    return metrics

def str_metric(metric_name, p, nb_scenario):
    # return "S" + str(nb_scenario) + "_" + metric_name + ":" +  str(p[str(nb_scenario)][metric_name]) + ":"
    return str(p[str(nb_scenario)][metric_name]).replace('.',',') + " "

def dump_data(dir_path):
    f = open(dir_path+'/metrics.txt', 'w')
    
    for p in overall_data:
        f.write(p["stamp"] + " ")
        for i in range(1,7):
            f.write(str_metric("task_completion_time", p, i))
            f.write(str_metric("number_steps", p, i))
            f.write(str_metric("nb_h_optimal_action", p, i))
            f.write(str_metric("ratio_h_optimal_action", p, i))
            f.write(str_metric("total_decision_time", p, i))
            f.write(str_metric("average_decision_time", p, i))
            f.write(str_metric("total_wait_ns", p, i))
            f.write(str_metric("average_wait_ns", p, i))
            f.write(str_metric("nb_h_action", p, i))
            f.write(str_metric("total_h_action_time", p, i))
            f.write(str_metric("average_h_action_time", p, i))
            f.write(str_metric("nb_r_action", p, i))
            f.write(str_metric("total_r_action_time", p, i))
            f.write(str_metric("average_r_action_time", p, i))

            # task_completion_time
            # number_steps
            # nb_h_optimal_action
            # total_decision_time
            # average_decision_time
            # total_wait_ns
            # average_wait_ns
            # nb_h_action
            # total_h_action_time
            # average_h_action_time
            # nb_r_action
            # total_r_action_time
            # average_r_action_time

        f.write("\n")

    f.close()


if __name__=='__main__':

    overall_data = []

    domain_name, g_tee_states, g_tee_final = dill.load(open(CM.path + "policy_task_end_early.p", "rb"))
    domain_name, g_hfe_states, g_hfe_final = dill.load(open(CM.path + "policy_real_human_free_early.p", "rb"))


    dir_path = '/home/afavier/new_exec_sim_ws/events'
    files = []
    for file_path in os.listdir(dir_path):
        if os.path.isfile(os.path.join(dir_path, file_path)):
            if file_path not in ['.gitignore', 'events.p', 'metrics.txt']:
                files.append(file_path)
    
    files.sort()
    for f in files:
        print(f + "\t" + f[f.find("_N")+4])

    i = 0
    participant_metrics = {}
    while i<len(files):
        f = files[i]
        id = f[f.find("_N")+4]

        if id=='t':
            if participant_metrics!={}:
                overall_data.append(participant_metrics)
                print("=> participant metrics saved")
            participant_metrics = {"stamp": f[:f.find("_N")]}
        else:
            print("Treating ", f)
            loading(dir_path+'/'+f)
            metrics = metrics_retrieval()
            participant_metrics[id] = metrics

        i+=1
        if i==len(files):
            overall_data.append(participant_metrics)

    dump_data(dir_path)



