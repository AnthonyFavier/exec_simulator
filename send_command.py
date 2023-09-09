#!/usr/bin/env python3
import rospy
import typing
from sim_msgs.msg import Action
import sys


rospy.init_node('send_command')

cmd_pub = rospy.Publisher('/robot_action', Action, queue_size=1)
rospy.sleep(0.1)

while not rospy.is_shutdown():

    print("cmd: ", end="")
    cmd = input()

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

    print(" ")
    rospy.sleep(0.1)
