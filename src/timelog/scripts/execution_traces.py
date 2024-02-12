#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Dict, List, Tuple
from copy import deepcopy
import dill
import sys
import os 

import timelog

path = "/home/afavier/ws/HATPEHDA/domains_and_results/"
sys.path.insert(0, path)
import ConcurrentModule as ConM
import CommonModule as CM


def loading(file_path):
    (timelog.g_events, timelog.g_r_signals, timelog.g_h_signals, timelog.g_to_signals) = dill.load(open(file_path, "rb"))
    timelog.g_h_activities = []
    timelog.g_r_activities = []
    print("\t events loaded")

    timelog.reset_times()
    timelog.r_extract_activities()
    timelog.h_extract_activities()

def extract_trace():
    trace = [0] # type: List[int]

    i_r = 0
    i_h = 0
    ps = CM.g_PSTATES[0]
    while i_r<len(timelog.g_r_activities) and i_h<len(timelog.g_h_activities):

        ## IDENTIFY EXECUTED PAIR ##
        # identify HA 
        if timelog.g_h_activities[i_h].name=="passive":
            ha_name = "Passive"
        elif timelog.g_h_activities[i_h].name=="start_delay":

            i_h+=1
            if timelog.g_h_activities[i_h].name=="pass":

                i_h+=1 
                if i_h>=len(timelog.g_h_activities) or timelog.g_h_activities[i_h].name in ["start_delay", "passive"]:
                    ha_name = "Passive"
                    i_h-=1

                elif timelog.g_h_activities[i_h].name not in timelog.g_h_activities_names:
                    ha_name = timelog.g_h_activities[i_h].name.replace('\n','')
                    i_h+=1

            elif timelog.g_h_activities[i_h].name not in timelog.g_h_activities_names:
                ha_name = timelog.g_h_activities[i_h].name.replace('\n','')
                i_h+=1

        i_h+=1 # next_step
    
        # identify RA
        while timelog.g_r_activities[i_r].name != "wait_end_step":
            i_r+=1
        if timelog.g_r_activities[i_r-1].name not in timelog.g_r_activities_names:
            ra_name = timelog.g_r_activities[i_r-1].name.replace('\n','')
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

        # update current pstate
        ps = CM.g_PSTATES[executed_pair.child]
        trace.append(ps.id)

    return trace

def trace_already_in(trace, traces):
    return trace in traces

if __name__=='__main__':


    domain_name, CM.g_PSTATES, CM.g_FINAL_IPSTATES = dill.load(open(CM.path + "search_space.p", 'rb'))

    dir_path = '/home/afavier/new_exec_sim_ws/events'
    files = []
    for file_path in os.listdir(dir_path):
        if os.path.isfile(os.path.join(dir_path, file_path)):
            if 'unedited' not in file_path:
                if file_path not in ['.gitignore', 'events.p', 'metrics.txt', 'optimal.txt']:
                    i_end_date = file_path.find('_')
                    date = file_path[:i_end_date]
                    d,m,y = date.split('-')
                    new_date = y+'-'+m+'-'+d
                    files.append( new_date + file_path[i_end_date:])
    
    files.sort()
    for f in files:
        print(f + "\t" + f[f.find("_N")+4])


    traces = []

    i = 0
    while i<len(files):

        f = files[i]
        id = f[f.find("_N")+4]

        if id=='t':
            i+=1
            continue

        i_end_date = f.find('_')
        new_date = f[:i_end_date]
        d,m,y = new_date.split('-')
        old_date = y+'-'+m+'-'+d
        new_name_file = old_date + f[i_end_date:]
        print("Treating ", new_name_file)
        loading(dir_path+'/'+new_name_file)

        new_trace = extract_trace()
        print("\t", new_trace)
        if not trace_already_in(new_trace, traces):
            traces.append(new_trace)

        i+=1
        

    print(f"traces (N={len(traces)}):")
    for t in traces:
        print("\t", t)





