#!/usr/bin/env python

import rospy
from std_msgs.msg import String 
from std_msgs.msg import Empty as EmptyM
import sys
from termios import tcflush, TCIFLUSH

def prompt_print(str):
  print('\033[2J')
  print(str)

def msg_cb(msg):
  prompt_print(msg.data)

def wait_press_enter_cb(msg):
  tcflush(sys.stdin, TCIFLUSH)
  input()
  g_enter_pressed_pub.publish(EmptyM())


if __name__ == "__main__":
  rospy.init_node("simulation_prompt")

  msg_sub = rospy.Subscriber("/simu_prompt", String, msg_cb, queue_size=1)

  wait_start_signal_sub = rospy.Subscriber("/wait_press_enter", EmptyM, wait_press_enter_cb, queue_size=1)
  g_enter_pressed_pub = rospy.Publisher('/enter_pressed', EmptyM, queue_size=1)


  print('\033[2J')
  rospy.spin()

