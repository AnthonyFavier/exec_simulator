from copy import deepcopy
from typing import Any, Dict, List
from enum import Enum
import sys

###############
## CONSTANTS ##
###############
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class DecompType(Enum):
    OK = 0
    NO_APPLICABLE_METHOD = 1
    AGENDA_EMPTY = 2
    BOTH_AGENDAS_EMPTY = 3
class OpType(Enum):
    NOT_APPLICABLE = 0
    DONE = 1

DEFAULT_ACTION_COST = 1.0
LRD_ACTION_COST = 1.0

#############
## CLASSES ##
#############
## Task ##
class Task:
    __ID = 0
    def __init__(self, name: str, parameters: list, is_abstract: bool, why, method_number: int, agent: str):
        self.id = Task.__ID
        Task.__ID += 1
        self.name = name
        self.parameters = parameters
        self.agent = agent

        # From which task it is decomposed, and how (number/id of method used)
        self.why = why  # From which task it is decomposed
        self.method_number = method_number

        self.is_abstract = is_abstract

        self.previous = None
        self.next = []
        self.current_plan_cost = -1.0

    def assign_next_id(self):
        self.id = Task.__ID
        Task.__ID += 1
    
    def __repr__(self):
        if not self.agent in [g_human_name, g_robot_name]:
           raise Exception("Unknown Agent")
        ag_str = "H" if self.agent==g_human_name else "R" 
        abs_str = "A" if self.is_abstract else "P"
        return "{}-{}{}-{}{}".format(self.id, ag_str, abs_str, self.name, self.parameters)

    def show(self):
        print(self)

class AbstractTask(Task):
    def __init__(self, name: str, parameters: list, why, method_number: int, agent: str):
        super().__init__(name, parameters, True, why, method_number, agent)

    def __repr__(self):
        if not self.agent in [g_human_name, g_robot_name]:
           raise Exception("Unknown Agent")
        ag_str = "H" if self.agent==g_human_name else "R" 
        return "{}-{}AT-{}{}".format(self.id, ag_str, self.name, self.parameters)

class Method:
    def __init__(self, AT_name, done_cond=None, pre_cond=None, decomp=None):
        self.AT_name = AT_name
        self.done_cond = done_cond
        self.pre_cond = pre_cond
        self.decomp = decomp

    def is_done(self, state, AT):
        return self.done_cond(state, AT.agent, *AT.parameters) if self.done_cond!=None else False

    def is_applicable(self, state, AT):
        return self.pre_cond(state, AT.agent, *AT.parameters) if self.pre_cond!=None else True
    
    def get_decomp(self, state, AT):
        return self.decomp(state, AT.agent, *AT.parameters) if self.decomp!=None else [] 

class PrimitiveTask(Task):
    def __init__(self, name: str, parameters: list, why, method_number: int, agent: str) -> None:
        super().__init__(name, parameters, False, why, method_number, agent)
    
    def __repr__(self):
        if not self.agent in [g_human_name, g_robot_name]:
           raise Exception("Unknown Agent")
        ag_str = "H" if self.agent==g_human_name else "R" 
        return "{}-{}PT-{}{}".format(self.id, ag_str, self.name, self.parameters)

class Operator:
    def __init__(self, PT_name, done_cond=None, pre_cond=None, effects=None, cost_function=None):
        self.PT_name = PT_name
        self.done_cond = done_cond
        self.pre_cond = pre_cond
        self.effects = effects
        self.cost_function = cost_function

    def is_done(self, state, PT):
        return self.done_cond(state, PT.agent, *PT.parameters) if self.done_cond!=None else False

    def is_applicable(self, state, PT):
        return self.pre_cond(state, PT.agent, *PT.parameters) if self.pre_cond!=None else True

    def get_cost(self, state, PT):
        return self.cost_function(state, PT.agent, *PT.parameters) if self.cost_function!=None else DEFAULT_ACTION_COST

    def apply_effects(self, state, agent: str, parameters):
        if self.effects!=None:
            self.effects(state, agent, *parameters)

    def apply(self, agents, PT):
        state = agents[PT.agent].state
        other_state = agents[g_other_agent_name[PT.agent]].state

        # Check Done and Pre conditions
        if self.is_done(state, PT):
            print(bcolors.WARNING + "already done!" + bcolors.ENDC)
            return OpType.DONE
        if not self.is_applicable(state, PT):
            print(bcolors.WARNING + " not applicable!" + bcolors.ENDC)
            return OpType.NOT_APPLICABLE

        # Compute cost
        cost = self.get_cost(state, PT)

        # Apply effects to acting and other agent beliefs
        self.apply_effects(state, PT.agent, PT.parameters)
        self.apply_effects(other_state, PT.agent, PT.parameters)

        return cost

class Action: #AppliedPrimitiveTask
    def __init__(self, cost: float):
        self.cost = cost

    def cast_PT2A(PT: PrimitiveTask, cost: float):
        """PT is modified!!"""
        PT.__class__ = Action
        PT.__init__(cost)
        return PT

    def create_wait(agent: str, why):
        return Action.cast_PT2A(PrimitiveTask("WAIT", (), why, 0, agent), g_wait_cost[agent])

    def create_idle(agent: str, why):
        return Action.cast_PT2A(PrimitiveTask("IDLE", (), why, 0, agent), g_idle_cost[agent])

    def create_LRD():
        return Action.cast_PT2A(PrimitiveTask("LRD", (), None, 0, g_human_name), LRD_ACTION_COST) 

    def are_similar(A1, A2):
        if A1.name==A2.name:
            if A1.parameters==A2.parameters:
                if A1.cost==A2.cost:
                    if A1.agent==A2.agent:
                        return True
        return False

    def short_str(self):
        if not self.agent in [g_human_name, g_robot_name]:
           raise Exception("Unknown Agent")
        ag_str = "H" if self.agent==g_human_name else "R" 
        return f"{ag_str}{self.id}"

    def __repr__(self):
        if not self.agent in [g_human_name, g_robot_name]:
           raise Exception("Unknown Agent")
        ag_str = "H" if self.agent==g_human_name else "R" 
        # return "{}-{}A-{}{}-{}".format(self.id, ag_str, self.name, self.parameters, self.cost)
        return "{}-{}A-{}{}".format(self.id, ag_str, self.name, self.parameters)

class Trigger:
    def __init__(self, pre_cond, decomp):
        self.pre_cond = pre_cond
        self.decomp = decomp

## State ##
class Fluent:
    def __init__(self, name, value, is_dyn):
        self.name = name
        self.val = value
        self.is_dyn = is_dyn

    def __repr__(self):
        return f"{self.val}"

class State:
    def __init__(self, name):
        self.__name__ = name
        self.fluent_names = []

    def create_static_fluent(self, name, value):
        self.fluent_names.append(name)
        setattr(self, name, Fluent(name, value, False))

    def create_dyn_fluent(self, name, value):
        self.fluent_names.append(name)
        setattr(self, name, Fluent(name, value, True))

    def get_fluent(self, fluent_name):
        return getattr(self, fluent_name)

## Agent ##
class Agent:
    def __init__(self, name):
        #####################
        # Static part
        self.name = name # type: str
        self.operators = {} # type: dict[str, Operator] # str=PT.name
        self.methods = {} # type: dict[str, list[Method]] # str=AT.name
        self.triggers = [] # type: list[Trigger]

        #####################
        # Dynamic part
        self.state = None # type: State
        self.agenda = [] # type: list[Task]
        self.planned_actions = [] # type: list[Action]
    
    #####################
    # Methods/Functions -> Static
    def show_planned_actions(self):
        print("{} planned actions:".format(self.name))
        for a in self.planned_actions:
            print("\t-{}".format(a))
    
    #####################
    # Deepcopy 
    def __deepcopy__(self, memo):
        # Mandatory: to match the static or dynamic fields in __init__ and deepcopy 
        cp = Agent(self.name)
        #####################
        # Static part
        cp.operators = self.operators
        cp.methods = self.methods
        cp.triggers = self.triggers

        #####################
        # Dynamic part
        cp.state = deepcopy(self.state)
        cp.agenda = deepcopy(self.agenda)
        cp.planned_actions = deepcopy(self.planned_actions)

        return cp

    def has_operator_for(self, PT):
        return PT.name in self.operators

    def has_method_for(self, AT):
        return AT.name in self.methods

class Agents:
    def __init__(self):
        self.agents = {} # type: dict[str, Agent]

    def exist(self, name):
        return name in self.agents
    
    def create_agent(self, name):
        if not self.exist(name): 
            self.agents[name] = Agent(name)

    def __getitem__(self, subscript):
        return self.agents[subscript]

    def __setitem__(self, subscript, item):
        self.agents[subscript] = item

    def __delitem__(self, subscript):
        del self.agents[subscript]

## Refinement ##
class Decomposition:
    def __init__(self, subtasks, agenda=None):
        self.type = DecompType.OK
        self.subtasks = subtasks
        self.new_agenda = agenda
        self.PT = None if subtasks==[] else subtasks[0]
        self.next_action = None 
    
    def show(self):
        print(self)

    def __str__(self):
        dec_str = "["
        if self.type != DecompType.OK:
            dec_str += str(self.type)
        else:
            for i, task in enumerate(self.subtasks):
                dec_str += str(task)
                if i < len(self.subtasks)-1:
                    dec_str += " - "
        dec_str += "]"
        return dec_str

    def first_task_is_PT_and_has_op(self, agent_name):
        # checks if first task is primitive and has an operator
        first_task = self.subtasks[0]
        
        if first_task.is_abstract==False and g_static_agents[agent_name].has_operator_for(first_task):
            self.PT = self.subtasks[0]
            return True
        return False

    def first_task_is_PT_not_done(self, agent_name, state):
        # checks if first task is primitive, has an operator, is not done
        if self.first_task_is_PT_and_has_op(agent_name):
            op = g_static_agents[agent_name].operators[self.subtasks[0].name]
            return not op.is_done(state, self.PT)
        return False
    
    def first_task_is_PT_done(self, agent_name, state):
        # checks if first task is primitive, has an operator, is done
        if self.first_task_is_PT_and_has_op(agent_name):
            op = g_static_agents[agent_name].operators[self.subtasks[0].name]
            return op.is_done(state, self.PT)
        return False

class AppliedDecomposition(Decomposition):
    def __init__(self, new_agents):
        self.next_action = None
        self.end_agents = new_agents

    def cast_Dec(dec: Decomposition, new_agents: Agents):
        dec.__class__ = AppliedDecomposition
        dec.__init__(new_agents)
        return dec

    def __str__(self):
        dec_str = "["
        if self.type != DecompType.OK:
            dec_str += str(self.type)
        else:
            dec_str += str(self.next_action) + " | "
            for i, task in enumerate(self.subtasks[1:]):
                dec_str += str(task)
                if i < len(self.subtasks)-1:
                    dec_str += " - "
            dec_str += " | "
            for i, task in enumerate(self.new_agenda):
                dec_str += str(task)
                if i < len(self.new_agenda)-1:
                    dec_str += " - "
        dec_str += "]"
        return dec_str

class Refinement:
    def __init__(self, decomp=None):
        self.decompos = [] # type: list[Decomposition]
        if decomp!=None:
            self.decompos.append(decomp)
        
    def add(self, decomp):
        self.decompos.append(decomp)
    
    def show(self):
        print("[\n", end="")
        for decomp in self.decompos:
            print("\t{}".format(decomp))
        print("]")

    def show_next_actions(self):
        print("next actions:")
        for decomp in self.decompos:
            if decomp.PT != None:
                print("\t- {}".format(decomp.PT))
        print("")

class AppliedRefinement:
    def __init__(self, refinement, agents) -> None:
        self.applied_decomps = [] #type: List[AppliedDecomposition]
        for dec in refinement.decompos:
            new_agents = deepcopy(agents)
            self.applied_decomps.append( AppliedDecomposition.cast_Dec(dec, new_agents) )

    def show(self):
        print("[\n", end="")
        for d in self.applied_decomps:
            print("\t{}".format(d))
        print("]")


##################################
## GLOBAL VARIABLES AND SETTERS ##
##################################
g_domain_name=""
def set_domain_name(dom_name):
    global g_domain_name
    g_domain_name = dom_name
g_static_agents = Agents()
g_robot_name=""
g_human_name=""
g_other_agent_name={}
def get_robot_name():
    return g_robot_name
def get_human_name():
    return g_human_name
def set_agents_name(robot_name, human_name):
    global g_robot_name, g_human_name
    g_robot_name = robot_name
    g_human_name = human_name
    g_other_agent_name[g_robot_name] = g_human_name
    g_other_agent_name[g_human_name] = g_robot_name
    init_inactivity_costs()
g_wait_cost = {}
g_idle_cost = {}
def init_inactivity_costs():
    g_wait_cost[g_robot_name]=0.0
    g_wait_cost[g_human_name]=2.0
    g_idle_cost[g_robot_name]=0.0
    g_idle_cost[g_human_name]=0.0
g_starting_agent = g_robot_name
def set_starting_agent(agent):
    global g_starting_agent
    g_starting_agent = agent
g_debug = False
def set_debug(val):
    global g_debug
    g_debug = val
g_compute_gui = False
def set_compute_gui(val):
    global g_compute_gui
    g_compute_gui = val
g_view_gui = False
def set_view_gui(val):
    global g_view_gui
    g_view_gui = val


###################
## INIT FUNCTION ##
###################
def declare_methods(agent, method_list):
    if not g_static_agents.exist(agent):
        g_static_agents.create_agent(agent)
    for t in method_list:
        g_static_agents[agent].methods[t[0]] = t[1:] 

def declare_operators(agent, op_list):
    if not g_static_agents.exist(agent):
        g_static_agents.create_agent(agent)
    for o in op_list:
        g_static_agents[agent].operators[o.PT_name] = o

def set_state(agent, state):
    if not g_static_agents.exist(agent):
        g_static_agents.create_agent(agent)
    g_static_agents[agent].state = state

def add_tasks(agent, tasks):
    if not g_static_agents.exist(agent):
        g_static_agents.create_agent(agent)

    for t in tasks:
        if t[0] in g_static_agents[agent].methods:
            g_static_agents[agent].agenda.append(AbstractTask(t[0], t[1:], None, 0, agent))
        elif t[0] in g_static_agents[agent].operators:
            g_static_agents[agent].agenda.append(PrimitiveTask(t[0], t[1:], None, 0, agent))
        else:
            raise Exception("{} isn't known by agent {}".format(t[0], agent))

def generate_begin_action():
    if g_starting_agent == g_robot_name or g_starting_agent=="":
        begin_agent = g_human_name
    elif g_starting_agent == g_human_name:
        begin_agent = g_robot_name
        
    begin_action = Action.cast_PT2A(PrimitiveTask("BEGIN", [], None, 0, begin_agent), 0.0)
    return begin_action


############
## PRINTS ##
############
# ─ ┌ ┐ ├ ┤ │ └ ┘
def show_init():
    print("┌────────────────────────────────────────────────────────────────────────┐")
    print("│ #INIT#                                                                 │")
    print("├────────────────────────────────────────────────────────────────────────┘")
    print_agendas_states(g_static_agents, with_static=True)

def str_init():
    out_str = ""
    out_str += "┌────────────────────────────────────────────────────────────────────────┐\n"
    out_str += "│ #INIT#                                                                 │\n"
    out_str += "├────────────────────────────────────────────────────────────────────────┘\n"
    out_str += str_agendas_states(g_static_agents, with_static=True)
    return out_str

def str_agents(agents):
    out_str = ""
    out_str += "┌────────────────────────────────────────────────────────────────────────┐\n"
    out_str += "│ #AGENTS#                                                               │\n"
    out_str += "├────────────────────────────────────────────────────────────────────────┘\n"
    out_str += str_agendas_states(agents)
    return out_str

def show_agents(agents):
    print("┌────────────────────────────────────────────────────────────────────────┐")
    print("│ #AGENTS#                                                               │")
    print("├────────────────────────────────────────────────────────────────────────┘")
    print_agendas_states(agents)

def str_agendas_states(agents, with_static=False):
    out_str = ""
    out_str += str_agendas(agents)
    out_str += "├─────────────────────────────────────────────────────────────────────────\n"
    out_str += "│ STATE =\n"
    out_str += str_state(agents[g_robot_name].state, with_static=with_static)
    out_str += "└─────────────────────────────────────────────────────────────────────────\n"
    return out_str

def print_agendas_states(agents, with_static=False):
    print_agendas(agents)
    print("├─────────────────────────────────────────────────────────────────────────")
    print("│ STATE =")
    print_state(agents[g_robot_name].state, with_static)
    print("└─────────────────────────────────────────────────────────────────────────")

def str_agendas(agents):
    out_str = ""
    out_str += str_agenda(agents[g_robot_name])
    out_str += str_agenda(agents[g_human_name])
    return out_str

def print_agendas(agents):
    print_agenda(agents[g_robot_name])
    print_agenda(agents[g_human_name])

def str_agenda(agent):
    out_str = ""
    out_str +=  "│ AGENDA {} =\n".format(agent.name)
    if len(agent.agenda)==0:
        out_str += "││\t*empty*\n"
    for t in agent.agenda:
        out_str += ("││\t{}\n".format(t))
    return out_str

def print_agenda(agent):
    print("│ AGENDA {} =".format(agent.name))
    if len(agent.agenda)==0:
        print("││\t*empty*")
    for t in agent.agenda:
        print("││\t-{}".format(t))

def str_state(state, indent=4, with_static=False):
    """Print each variable in state, indented by indent spaces."""
    out_str = ""
    if state != False:
        for fluent_name in state.fluent_names:
            fluent = state.get_fluent(fluent_name)
            if fluent.is_dyn or fluent.is_dyn==False and with_static:
                out_str += "││"
                for x in range(indent): out_str += " "
                out_str += f"{fluent.name}"
                out_str += ' = {}\n'.format(fluent.val)
    else:
        out_str += 'False\n'
    return out_str

def print_state(state, indent=4, with_static=False):
    """Print each variable in state, indented by indent spaces."""
    if state != False:
        for fluent_name in state.fluent_names:
            fluent = state.get_fluent(fluent_name)
            if fluent.is_dyn or fluent.is_dyn==False and with_static:
                print("││", end='')
                for x in range(indent): sys.stdout.write(' ')
                sys.stdout.write(state.__name__ + '.' + fluent.name)
                print(' = {}'.format(fluent.val))
    else:
        print('False')

def print_solutions(begin_action: Action):
    print("Solutions:")
    last_actions = get_last_actions(begin_action)

    plans = []

    for last_action in last_actions:
        plan=[]
        action = last_action
        while action != begin_action:
            plan.append(action)
            action = action.previous
        plans.append(plan)

    for plan in plans:
        print("\nPLAN:")
        n=0
        for x in plan:
            n-=1
            print(plan[n], end=" - ") if x!=plan[-1] else print(plan[n])


############
## HELPER ##
############

def get_last_actions(begin_action: Action):
    if begin_action.next==[]:
        return [begin_action]
    else:
        last_actions = []
        for next in begin_action.next:
            last_actions += get_last_actions(next)

        return last_actions
