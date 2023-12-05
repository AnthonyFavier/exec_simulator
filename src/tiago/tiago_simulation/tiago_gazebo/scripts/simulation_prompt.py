#!/usr/bin/env python

import rospy
from std_msgs.msg import String 
from std_msgs.msg import Empty as EmptyM
from std_srvs.srv import Empty as EmptyS
from std_srvs.srv import EmptyResponse
import sys
from termios import tcflush, TCIFLUSH

def prompt_print(str):
  print('\033[2J')
  print(str, end='', flush=True)

def show_progress_bar_cb(s):
  print("\r" + s.data, end='')

def start_prompt_bar_cb(msg):
  print("\n", end="", flush=True)

def msg_cb(msg):
  prompt_print(msg.data)

def wait_press_enter_cb(msg):
  tcflush(sys.stdin, TCIFLUSH)
  input()
  g_enter_pressed_pub.publish(EmptyM())


if __name__ == "__main__":
  rospy.init_node("simulation_prompt")

  msg_sub = rospy.Subscriber("/simu_prompt", String, msg_cb, queue_size=1)
  update_progress_bar_sub = rospy.Subscriber('/prompt_update_progress_bar', String, show_progress_bar_cb, queue_size=1)
  start_prompt_bar_sub = rospy.Subscriber('/start_prompt_bar', EmptyM, start_prompt_bar_cb, queue_size=1)

  wait_start_signal_sub = rospy.Subscriber("/wait_press_enter", EmptyM, wait_press_enter_cb, queue_size=1)
  g_enter_pressed_pub = rospy.Publisher('/enter_pressed', EmptyM, queue_size=1)


  print('\033[2J')

  started_service = rospy.Service("prompt_started", EmptyS, lambda req: EmptyResponse())


  rospy.spin()

