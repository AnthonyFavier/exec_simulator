#!/usr/bin/env python3
import rospy
import typing
from sim_msgs.msg import Action
import sys

# stack_empiler | stack_empiler_1 | stack_empiler_2 | stack_box
DOMAIN_NAME = "stack_empiler_2"

rospy.init_node('send_command')

cmd_pub = rospy.Publisher('/robot_action', Action, queue_size=1)
rospy.sleep(0.1)


def treat_input_empiler_1(cmd):
    return treat_input_empiler(cmd)

def treat_input_empiler_2(cmd):
    return treat_input_empiler(cmd)

def treat_input_stack_box(cmd):
    return treat_input_empiler(cmd)

def treat_input_empiler(cmd):
    ## Pick
    if cmd=="p":
        print("obj_name: ", end="")
        obj_name = input()
        msg = Action()
        msg.type = Action.PICK_OBJ_NAME
        msg.obj_name = obj_name
        cmd_pub.publish(msg)

    ## Place
    elif cmd=="pl":
        print("loc: ", end="")
        loc = input()
        if loc in ['l1','l2','l3','l4','l5']:
            msg = Action()
            msg.type = Action.PLACE_OBJ_NAME
            msg.location = loc
            cmd_pub.publish(msg)
        else:
            raise Exception("Wrong location")

    ## Drop
    elif cmd=="d":
        msg = Action()
        msg.type = Action.DROP
        cmd_pub.publish(msg)

    ## Open Box
    elif cmd=="o":
        msg = Action()
        msg.type = Action.OPEN_BOX
        cmd_pub.publish(msg)

def treat_input_classic(cmd):
     ## Pick
    if cmd=="p":
        print("color: ", end="")
        color = input()
        if color in ['r','y','p','w','b']:
            print("side: ", end="")
            side = input()
            if side in ['R', 'C', 'H']:
                msg = Action()
                msg.type = Action.PICK_OBJ
                msg.color = color
                msg.side = side
                cmd_pub.publish(msg)
            else:
                raise Exception("Wrong side")
        else:
            raise Exception("Wrong color")

    ## Place
    elif cmd=="pl":
        print("loc: ", end="")
        loc = input()
        if loc in ['l1','l2','l3','l4','l5']:
            msg = Action()
            msg.type = Action.PLACE_OBJ
            msg.location = loc
            cmd_pub.publish(msg)
        else:
            raise Exception("Wrong location")

    ## End
    elif cmd=="q":
        print("quitting...")
        exit()

    else:
        raise Exception("Unknown command...")

while not rospy.is_shutdown():

    print("cmd: ", end="")
    cmd = input()

    ## End
    if cmd=="q":
        print("quitting...")
        exit()

    else:
        if DOMAIN_NAME=="stack_empiler":
            treat_input_empiler(cmd)
        elif DOMAIN_NAME=="stack_empiler_1":
            treat_input_empiler_1(cmd)
        elif DOMAIN_NAME=="stack_empiler_2":
            treat_input_empiler_2(cmd)
        elif DOMAIN_NAME=="stack_box":
            treat_input_stack_box(cmd)
        elif DOMAIN_NAME=="new_stack":
            treat_input_classic(cmd)
        else:
            raise Exception("Unknown domain_name...")

    print(" ")
    rospy.sleep(0.1)
