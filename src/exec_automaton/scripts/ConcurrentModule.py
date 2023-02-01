from typing import Any, Dict, List, Tuple
from copy import deepcopy
import CommonModule as CM
from anytree import RenderTree, NodeMixin
import pickle
import dill
import logging as lg
import logging.config
import sys

import graphviz

sys.path.insert(0, "/home/afavier/exec_simulator_ws/src/exec_automaton/scripts")
logging.config.fileConfig('/home/afavier/exec_simulator_ws/src/exec_automaton/scripts/log.conf')

#############
## CLASSES ##
#############
class ActionPair:
    def __init__(self, human_action: CM.Action, robot_action: CM.Action, end_agents: CM.Agents):
        self.human_action = human_action    #type: ActionPair
        self.robot_action = robot_action    #type: ActionPair
        self.previous = None                #type : ActionPair | None
        self.next = []                      #type: List[ActionPair]
        self.end_agents = end_agents        #type: CM.Agents
        self.in_human_option = None         #type: HumanOption | None

    def get_short_str(self):
        return f"{self.human_action.short_str()}{self.robot_action.short_str()}"

    def get_in_step(self):
        return self.in_human_option.in_step

    def __repr__(self):
        return f"H{self.human_action.id}-{self.human_action.name}{self.human_action.parameters}|R{self.robot_action.id}-{self.robot_action.name}{self.robot_action.parameters}"

class HumanOption:
    def __init__(self, pairs: List[ActionPair]):
        self.action_pairs = pairs                   #type: List[ActionPair]
        self.in_step = None                         #type: Step | None
        self.human_action = pairs[0].human_action   #type: CM.Action
        self.robot_actions = []                     #type: List[CM.Action]
        # inits
        for p in pairs:
            self.robot_actions.append(p.robot_action)
            p.in_human_option = self

    def get_str(self, SRA=None):
        # ─ ┌ ┐ ├ ┤ │ └ ┘
        str_h = f"┌H{self.human_action.id}-{self.human_action.name}{self.human_action.parameters}┐"
        str_r = "│"
        sra_present = False
        for i, ra in enumerate(self.robot_actions):
            if SRA!=None and CM.Action.are_similar(SRA, ra):
                str_r += CM.bcolors.BOLD + f"R{ra.id}-{ra.name}{ra.parameters}" + CM.bcolors.ENDC + "│"
                sra_present=True
            else:
                str_r += f"R{ra.id}-{ra.name}{ra.parameters}│"

        l_h = len(str_h)
        l_r = len(str_r)
        if SRA!=None and sra_present:
            l_r -= 8 # 8 = len(CM.bcolors.BOLD) + len(CM.bcolors.ENDC)

        if l_h<l_r:
            diff = l_r-l_h
            padding = ""
            for i in range(int(diff/2)):
                padding+="─"
            
            str_h = str_h[1:-1]
            if diff%2==1:
                str_h = "┌" + padding + str_h + padding + "─┐"
            else:
                str_h = "┌" + padding + str_h + padding + "┐"
        elif l_h>l_r:
            str_r = str_r[:-1]
            diff = l_h-l_r
            padding_f = ""
            for i in range(diff//2):
                padding_f+=" "
            padding_e = ""
            for i in range(diff-diff//2):
                padding_e+=" "
            str_r = str_r[0] + padding_f + str_r[1:] + padding_e + "│"

        if SRA!=None:
            l_end = max(len(str_h), len(str_r)-8)-2
        else:
            l_end = max(len(str_h), len(str_r))-2
        str_end = ""
        for i in range(l_end):
            str_end+="─"
        str_end = "└" + str_end + "┘"

        return str_h, str_r, str_end
        
    def show(self):
        str_1, str_2, str_3 = self.get_str()
        print(str_1)
        print(str_2)
        print(str_3)

class BaseStep:
    __ID = 0 #type: int
    def __init__(self, human_options: List[HumanOption], from_pair: ActionPair):
        self.id = BaseStep.__ID
        BaseStep.__ID += 1
        self.human_options = human_options  #type: List[HumanOption]
        self.SRA = None                     #type: CM.Action | None # Safe Robot Action
        self.CRA = []                       #type: List[CM.Action] # Common Robot Actions
        self.is_final = False
        self.from_pair = from_pair
        # inits
        for ho in human_options:
            ho.in_step = self

        pairs = self.get_pairs()
        if len(pairs)==1\
            and pairs[0].human_action.name=="IDLE"\
            and pairs[0].robot_action.name=="IDLE":
            self.is_final = True

        # Set next/previous of each pairs from new_step
        # Connect new pairs and expanded/select pair
        if from_pair!=None:
            all_step_pairs = self.get_pairs()
            from_pair.next += all_step_pairs
            for p in all_step_pairs:
                p.previous = from_pair

    def get_pairs(self) -> List[ActionPair]:
        pairs = [] #type: List[ActionPair]
        for ha in self.human_options:
            pairs += ha.action_pairs
        return pairs

    def show(self, last_line=True):
        a_from = "" if self.from_pair==None else f"-{self.from_pair.get_short_str()}"
        print(f"========= Step{self}{a_from} =========")
        ho_l1, ho_l2, ho_l3 = [], [], []
        for ho in self.human_options:
            l1, l2, l3 = ho.get_str(self.SRA)
            ho_l1.append(l1)
            ho_l2.append(l2)
            ho_l3.append(l3)
        str_1, str_2, str_3 = "", "", ""
        for i in range(len(self.human_options)):
            str_1 += ho_l1[i] + " "
            str_2 += ho_l2[i] + " "
            str_3 += ho_l3[i] + " "
        print(str_1)
        print(str_2)
        print(str_3)
        if last_line:
            print(f"===============================")

    def str(self, last_line=True):
        out_str = ""
        a_from = "" if self.from_pair==None else f"-{self.from_pair.get_short_str()}"
        out_str += f"========= Step{self}{a_from} =========\n"
        ho_l1, ho_l2, ho_l3 = [], [], []
        for ho in self.human_options:
            l1, l2, l3 = ho.get_str(self.SRA)
            ho_l1.append(l1)
            ho_l2.append(l2)
            ho_l3.append(l3)
        str_1, str_2, str_3 = "", "", ""
        for i in range(len(self.human_options)):
            str_1 += ho_l1[i] + " "
            str_2 += ho_l2[i] + " "
            str_3 += ho_l3[i] + " "
        out_str += str_1+"\n"
        out_str += str_2+"\n"
        out_str += str_3+"\n"
        if last_line:
            out_str += f"===============================\n"
        return out_str


    def get_str(self, with_bold=True):
        start_flags = ""
        end_flags = ""
        if self.is_final and with_bold:
            start_flags = CM.bcolors.BOLD + CM.bcolors.OKBLUE
            end_flags = CM.bcolors.ENDC
        return start_flags + f"({self.id})" + end_flags

    def __repr__(self) -> str:
        return self.get_str()

class Step(BaseStep, NodeMixin):  # Add Node feature
    def __init__(self, *params, parent=None, children=None):
        super(Step, self).__init__(*params)
        self.parent = parent
        if children:
            self.children = children


#############
## EXPLORE ##
#############
def explore():
    lg.info(CM.str_init())

    # Generate initial step
    # init agents
    initial_agents = deepcopy(CM.g_static_agents)
    # init APair
    begin_action_R = CM.Action.cast_PT2A(CM.PrimitiveTask("BEGIN", (), None, 0, CM.g_robot_name), 0.0)
    begin_action_H = CM.Action.cast_PT2A(CM.PrimitiveTask("BEGIN", (), None, 0, CM.g_human_name), 0.0)
    init_pair = ActionPair(begin_action_H, begin_action_R, initial_agents)
    # init HOption
    init_hoption = HumanOption([init_pair])
    # init Step
    init_step = Step([init_hoption], None)
    init_step.CRA = begin_action_R
    init_step.SRA = begin_action_R
    lg.debug(f"{init_step.str()}")

    # Several exploration steps
    pairs_to_explore = [] # order=priority
    pairs_to_explore.append(init_pair)
    while pairs_to_explore!=[]:
        lg.debug(f"\nNEW STEP:\npairs to explore:\n\t{pairs_to_explore}")
        pairs_to_explore = exploration_step(pairs_to_explore)
        lg.debug(RenderTree(init_step))

    #TEST GRAPHVIZ
    # render_dot(init_step)

    return init_step

def exploration_step(pairs_to_explore):
    selected_pair = select_pair_to_explore(pairs_to_explore)
    parallel_pairs = compute_parallel_pairs(selected_pair)
    human_options = arrange_pairs_in_HumanOption(parallel_pairs)
    new_step = Step(human_options, selected_pair, parent=selected_pair.get_in_step())
    new_step.SRA = find_safe_robot_action(new_step, selected_pair)
    lg.debug(f"{new_step.str()}")

    return get_pairs_to_explore(new_step, pairs_to_explore)

### 1 ###
def select_pair_to_explore(pairs_to_explore):
    selected_pair = pairs_to_explore.pop(0)
    lg.debug(f"Selected pair: {selected_pair} (from step {selected_pair.get_in_step()})")
    lg.debug(CM.str_agents(selected_pair.end_agents))
    return selected_pair

### 2 ###
def compute_parallel_pairs(selected_pair: ActionPair) -> List[ActionPair]:
    #2# Compute parallel pairs (and L.R.D. pairs)
    
    # Compute H starting pairs
    HS_pairs = []
    HS_applied_ref_h = get_applied_refinement(CM.g_human_name, selected_pair.end_agents)
    for h_ap_dec in HS_applied_ref_h.applied_decomps:
        HS_applied_ref_r = get_applied_refinement(CM.g_robot_name, h_ap_dec.end_agents)
        for r_ap_dec in HS_applied_ref_r.applied_decomps:
            pair = ActionPair(h_ap_dec.next_action, r_ap_dec.next_action, r_ap_dec.end_agents)
            HS_pairs.append(pair)
    
    # Compute R starting pairs
    RS_pairs = []
    RS_applied_ref_r = get_applied_refinement(CM.g_robot_name, selected_pair.end_agents)
    for r_ap_dec in RS_applied_ref_r.applied_decomps:
        RS_applied_ref_h = get_applied_refinement(CM.g_human_name, r_ap_dec.end_agents)
        for h_ap_dec in RS_applied_ref_h.applied_decomps:
            pair = ActionPair(h_ap_dec.next_action, r_ap_dec.next_action, h_ap_dec.end_agents)
            RS_pairs.append(pair)
    
    # Check if both agents are active
    h_active = check_list(HS_pairs, lambda x: not x.human_action.name in ["WAIT","IDLE"])!=None
    r_active = check_list(RS_pairs, lambda x: not x.robot_action.name in ["WAIT","IDLE"])!=None
    agents_active = h_active and r_active

    # Create LRD pairs
    lrd_pairs = []
    lrd_action = CM.Action.create_LRD()
    if agents_active:
        for RS_pair in RS_pairs:
            already_exist = check_list(lrd_pairs, lambda x: CM.Action.are_similar(x.robot_action, RS_pair.robot_action))!=None
            if not already_exist:
                new_agents = get_agents_after_action(deepcopy(selected_pair.end_agents), RS_pair.robot_action) 
                new_agents[CM.g_human_name].planned_actions.append( lrd_action )
                new_lrd_pair = ActionPair(lrd_action, RS_pair.robot_action, new_agents)
                lrd_pairs.append(new_lrd_pair)

    # Parallel pairs
    parallel_pairs = []
    for HS_pair in HS_pairs:
        found = check_list(RS_pairs, lambda x: CM.Action.are_similar(HS_pair.robot_action, x.robot_action)\
                and CM.Action.are_similar(HS_pair.human_action, x.human_action))!=None
        if found:
            parallel_pairs.append(HS_pair)
        else:
            # Check if HA/WAIT already exist
            exist = check_list(parallel_pairs, lambda x: CM.Action.are_similar(x.human_action, HS_pair.human_action)\
                and x.robot_action.name=="WAIT")!=None
            if not exist:
                # Replace RA with WAIT and append
                r_wait_action = CM.Action.create_wait(CM.g_robot_name, None)
                new_agents = get_agents_after_action(deepcopy(selected_pair.end_agents), HS_pair.human_action)
                new_agents[CM.g_robot_name].planned_actions.append( r_wait_action )
                new_wait_pair = ActionPair(HS_pair.human_action, r_wait_action, new_agents)
                parallel_pairs.append(new_wait_pair)
    
    return parallel_pairs + lrd_pairs

### 3 ###
def arrange_pairs_in_HumanOption(parallel_pairs: List[ActionPair]) -> List[HumanOption]:
    #3# Arrange all pairs into HumanOptions
    raw_options = {}
    for p in parallel_pairs:
        if not p.human_action in raw_options:
            raw_options[p.human_action] = [p]
        else:
            raw_options[p.human_action].append(p)
    
    human_options = []
    for raw_o in raw_options:
        ha = HumanOption(raw_options[raw_o])
        human_options.append(ha)

    return human_options

### 4 ###
def find_safe_robot_action(new_step: Step, selected_pair: ActionPair) -> CM.Action:
    #4# Find Safe Robot Action to execute in this step

    # Identify C.R.A. (Common Robot Actions)
    cra_candidates = new_step.human_options[0].robot_actions[:]
    for ho in new_step.human_options[1:]:
        j = 0
        while j<len(cra_candidates):
            candidate = cra_candidates[j]

            found = check_list(ho.robot_actions, lambda x: CM.Action.are_similar(candidate, x))!=None
            if not found:
                cra_candidates.pop(j)
            else:
                j+=1
    CRA = cra_candidates
    new_step.CRA = CRA

    # If there is at least one CRA, select one as SRA
    if len(CRA)>0:
        SRA = CRA[0] # For now, select first one

    # If there is no CRA, insert WAIT in every human options, except LRD
    # Select WAIT as SRA
    elif len(CRA)==0:
        wait_action = CM.Action.create_wait(CM.g_robot_name, None)
        for ho in new_step.human_options:
            if ho.human_action.name=="LRD":
                continue
            already_a_robot_wait = check_list(ho.robot_actions, lambda x: CM.Action.are_similar(wait_action, x))!=None
            if already_a_robot_wait:
                continue

            new_agents = get_agents_after_action(selected_pair.end_agents, ho.human_action)
            new_agents[CM.g_robot_name].planned_actions.append( wait_action )
            wait_pair = ActionPair(ho.human_action, wait_action, new_agents)
            wait_pair.in_human_option = ho
            
            ho.robot_actions.append(wait_action)
            ho.action_pairs.append(wait_pair)
        SRA = wait_action

    return SRA

### 5 ###
def get_pairs_to_explore(new_step: Step, previous_pairs_to_explore: List[ActionPair]):
    # Criteria: All ActionPairs including the SRA must be explored. (priority)
    # Criteria: If SRA=WAIT, all pairs from the step must be explored (secondly, since we try to avoid plans with WAITs)
    prio_pairs = []
    no_prio_pairs = []
    if new_step.SRA.name=="WAIT":
        # All pairs must be explored
        for ho in new_step.human_options:
            no_prio_pairs += ho.action_pairs
    elif len(new_step.get_pairs())==1\
        and new_step.get_pairs()[0].human_action.name=="IDLE"\
        and new_step.get_pairs()[0].robot_action.name=="IDLE":
        pass
    else:
        # SRA pairs must be explored
        for ho in new_step.human_options:
            for p in ho.action_pairs:
                if CM.Action.are_similar(p.robot_action, new_step.SRA):
                    prio_pairs.append(p)
    pairs_to_explore = prio_pairs + previous_pairs_to_explore + no_prio_pairs

    return pairs_to_explore


################
## REFINEMENT ##
################
def get_applied_refinement(agent_name, agents):
    """
    Refines agent's agenda and applies it.
    I.e. the PT of each decomp is applied and inactivity actions may be inserted.
    Triggers are checked here.
    """

    # print("{}- Refine agenda".format(agent_name))
    refinement = refine_agenda(agent_name, agents)
    # print("refinement = ")
    # refinement.show()

    applied_ref = CM.AppliedRefinement(refinement, agents)

    for ap_dec in applied_ref.applied_decomps:
        if not CM.DecompType.OK == ap_dec.type:
            # TODO check if no triggers are forgotten when creating inactivity actions
            if CM.DecompType.NO_APPLICABLE_METHOD == ap_dec.type:
                print("NO_APPLICABLE_METHOD => WAIT added")
                action = CM.Action.create_wait(agent_name, None)
                ap_dec.type = CM.DecompType.OK
            elif CM.DecompType.AGENDA_EMPTY == ap_dec.type:
                # print("AGENDA_EMPTY => IDLE added")
                action = CM.Action.create_idle(agent_name, None)
                ap_dec.type = CM.DecompType.OK
        elif CM.DecompType.OK == ap_dec.type:
            # Apply PT operator's effects to both beliefs
            # Get PT operator
            if not CM.g_static_agents[agent_name].has_operator_for(ap_dec.PT):
                raise Exception("Agent {} doesn't have an operator for {}".format(agent_name, ap_dec.PT))
            op = CM.g_static_agents[agent_name].operators[ap_dec.PT.name]
            # Apply operator to both beliefs
            result = op.apply(ap_dec.end_agents, ap_dec.PT)
            if CM.OpType.NOT_APPLICABLE == result:
                print(str(ap_dec.PT) + " not applicable... WAIT inserted")
                ap_dec.new_agenda = [ap_dec.PT] + ap_dec.new_agenda
                action = CM.Action.create_wait(agent_name, ap_dec.PT)
            else:
                action = CM.Action.cast_PT2A(ap_dec.PT, result)

            check_triggers(ap_dec.end_agents)

        ap_dec.next_action = action
        ap_dec.end_agents[agent_name].agenda = ap_dec.subtasks[1:] + ap_dec.new_agenda
        ap_dec.end_agents[agent_name].planned_actions.append( action )

    return applied_ref

def refine_agenda(agent_name, in_agents):
    """
    Refines the agenda of the given agent until reaching a primitive task.
    Return a refinement, including decompositions for each applied different method.
    """

    static_agent = CM.g_static_agents[agent_name]
    state = deepcopy(in_agents[agent_name].state)
    new_agenda = in_agents[agent_name].agenda[:]

    refinement = CM.Refinement()
    refinement.add(CM.Decomposition([]))    

    # Check if agenda is empty (no task to refine)
    if new_agenda == []:
        refinement.decompos[0].new_agenda = []
        refinement.decompos[0].type = CM.DecompType.AGENDA_EMPTY
    else:
        next_task = new_agenda.pop(0)
        # print("Task to refine: {}".format(next_task))
        
        refinement.decompos[0].subtasks = [next_task]
        refinement.decompos[0].new_agenda = new_agenda

        i=0
        # While we didn't reach the end of each decomposition, we start with one decomposition
        while i<len(refinement.decompos):
            current_decomp = refinement.decompos[i]
            # print("decomp i= {}".format(i))
            # While first subtask of current decomposition isn't a PT we keep refining
            while not current_decomp.first_task_is_PT_not_done(agent_name, state):
                task = current_decomp.subtasks[0]
                next_subtasks = current_decomp.subtasks[1:]

                # Either already refine the task with methods or 
                if task.is_abstract and static_agent.has_method_for(task):
                    list_decompo = refine_method(task, state, new_agenda)
                elif not task.is_abstract and static_agent.has_operator_for(task): 
                    list_decompo = [CM.Decomposition([task], agenda=new_agenda)]
                else:
                    raise Exception("task={} can't be handled by agent {}.".format(task, agent_name))

                # END, If no method is applicable
                if list_decompo == []:
                    current_decomp.subtasks = next_subtasks
                    current_decomp.new_agenda = [task] + new_agenda
                    current_decomp.type = CM.DecompType.NO_APPLICABLE_METHOD
                    print("\t{} => {}".format(task, current_decomp.type))
                    break
                # There are applicable methods
                else:
                    # If we continue to refine with next tasks
                    need_continue = False
                    # If current method refines into nothing
                    if list_decompo[0].subtasks==[]:
                        print(CM.bcolors.OKGREEN + "\trefines into nothing" + CM.bcolors.ENDC)
                        need_continue = True
                    # If next task is an operator already done
                    elif current_decomp.first_task_is_PT_done(agent_name, state): # CHECK list_decomp ? 
                        print(CM.bcolors.OKGREEN + "\talready done" + CM.bcolors.ENDC)
                        need_continue = True
                    if need_continue:
                        # print("NEED_CONTINUE")
                        # If there are other subtasks we continue by refining the next one
                        if len(next_subtasks)>0:
                            print("\tcontinue .. next_subtasks={}".format(next_subtasks))
                            current_decomp.subtasks = next_subtasks
                        # No other subtasks, we have to pop the next task in agenda to continue
                        else:
                            # END, If the agendas are empty
                            if new_agenda==[]:
                                current_decomp.subtasks = next_subtasks
                                current_decomp.type = CM.DecompType.AGENDA_EMPTY
                                print("\t{} => {}".format(task, current_decomp.type))
                                break
                            # Agenda isn't empty, we can continue with next task in agenda
                            else:
                                next_task = new_agenda.pop(0)
                                print("\tcontinue with next_task popped from agenda {}".format(next_task))
                                current_decomp.subtasks = [next_task]
                    # Update subtasks list and add new decompositions for each additional applicable methods
                    else:
                        current_decomp.subtasks = list_decompo[0].subtasks + next_subtasks
                        for j in range(1, len(list_decompo)):
                            j_subtasks = list_decompo[j].subtasks + next_subtasks
                            # print("\t\tdecomposition {} : created : {}".format(j, j_subtasks))
                            refinement.add(CM.Decomposition(j_subtasks, agenda=new_agenda))

            # End of the decomposition reached, we look for the next one
            # print("\tend {}".format(i))
            if len(current_decomp.subtasks)>0:
                current_decomp.PT = current_decomp.subtasks[0]
            i+=1

    return refinement

def refine_method(task_to_refine, state, new_agenda):
    agent_name = task_to_refine.agent
    static_agent = CM.g_static_agents[agent_name]

    # get methods
    if not static_agent.has_method_for(task_to_refine):
        raise Exception("{} has no methods for {}".format(agent_name, task_to_refine))
    methods = static_agent.methods[task_to_refine.name]

    # apply each method to get decompositions
    list_decomps = []
    for i,m in enumerate(methods):
        if m.is_applicable(state, task_to_refine):
            dec_tuple = m.get_decomp(state, task_to_refine) # list(tuple(task_name: str, *params: Any))
            subtasks = []
            for tuple in dec_tuple:
                task = tuple[0]
                params = tuple[1:]
                if task in static_agent.methods:
                    subtasks.append(CM.AbstractTask(task, params, task_to_refine, i, agent_name))
                elif task in static_agent.operators:
                    subtasks.append(CM.PrimitiveTask(task, params, task_to_refine, i, agent_name))
                else:
                    raise Exception("{} isn't known by agent {}".format(task, agent_name))

            list_decomps.append(CM.Decomposition(subtasks, agenda=new_agenda))

    return list_decomps

def check_triggers(agents):
    #TODO
    pass

def get_agents_after_action(in_agents, action):
    ap_ref = get_applied_refinement(action.agent, in_agents)
    dec = check_list(ap_ref.applied_decomps, lambda x: CM.Action.are_similar(x.next_action, action))
    if dec==None:
        raise Exception("Corresponding decomposition not found!")
    return dec.end_agents


############
## HELPER ##
############
def check_list(list, cond):
    for x in list:
        if cond(x):
            return x


############
## RENDER ##
############
def render_dot(init_step: Step):
    g = graphviz.Digraph('G', filename='render_dot.gv')
    g.attr(compound='true')
    # g.edge_attr["minlen"]="2"

    steps_to_render = [init_step]

    begin_node_name=""

    i_cluster=0
    while steps_to_render!=[]:
        s = steps_to_render.pop(0)

        if s.is_root:
            begin_node_name = s.human_options[0].action_pairs[0].get_short_str()
            g.node(begin_node_name, shape='circle', style="filled", color="black", label="", width="0.2", fixedsize="true")
        elif s.is_final:
            name = "f_"+s.human_options[0].action_pairs[0].get_short_str()
            g.node(name, shape='doublecircle', style="filled", color="black", label="", xlabel="("+str(s.id)+")", width="0.2", fixedsize="true")
            if s.from_pair!=None:
                g.edge(str(s.from_pair.get_short_str()), name)
        else:
            name_step_cluster = f"cluster_{i_cluster}"
            one_node=""
            with g.subgraph(name=name_step_cluster) as cs:
                i_cluster+=1
                cs.attr(style='solid', bgcolor="#f3f3f3", label=f"Step{s.get_str(with_bold=False)}")
                for ho in s.human_options:
                    with cs.subgraph(name=f"cluster_{i_cluster}") as c:
                        i_cluster+=1
                        if s.is_final:
                            c.attr(label=str(ho.human_action), style='rounded', color="#4f984b", bgcolor="#aaeaa7")
                        else:
                            c.attr(label=str(ho.human_action), style='rounded', color="#D6B656", bgcolor="#FFE6CC")
                        
                        for p in ho.action_pairs:
                            node_name = p.get_short_str()
                            style="filled,solid"
                            if CM.Action.are_similar(p.robot_action,s.SRA):
                                style="filled,bold"
                            c.node(node_name, label=str(p.robot_action), style=style, color="#6C8EBF", fillcolor="#DAE8FC")
                            if one_node=="":
                                one_node=node_name

            if s.from_pair!=None:
                if s.from_pair.get_in_step().is_root:
                    g.edge(str(s.from_pair.get_short_str()), one_node, lhead=name_step_cluster, minlen="2")
                else:
                    g.edge(str(s.from_pair.get_short_str()), one_node, lhead=name_step_cluster)

        next_pairs = []
        for p in s.get_pairs():
            if p.next!=[]:
                next_pairs+=p.next
        for np in next_pairs:
            in_s = np.get_in_step()
            if not in_s in steps_to_render:
                steps_to_render.append(in_s)

    g.view()


###########
## PRINT ##
###########
def show_solution(init_step: Step):
    lg.info(f"\n### SOLUTION ### [domain:{CM.g_domain_name}]")
    lg.info(RenderTree(init_step))
    for s in init_step.descendants:
        if not s.is_leaf:
            lg.info(f"{s.str(last_line=False)}")

def dumping_solution(init_step: Step):
    whole_tree=(init_step, *init_step.descendants)
    dom_n_sol = (CM.g_domain_name, whole_tree)
    # pickle.dump(dom_n_sol, open("/home/afavier/exec_simulator_ws/src/exec_automaton/scripts/dom_n_sol.p", "wb"))
    dill.dump(dom_n_sol, open("/home/afavier/exec_simulator_ws/src/exec_automaton/scripts/dom_n_sol.p", "wb"))