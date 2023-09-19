#!/usr/bin/env python

import rospy
from std_msgs.msg import String 

def msg_cb(msg):
  print('\033[2J')
  print(msg.data)

if __name__ == "__main__":
  rospy.init_node("simulation_prompt")

  msg_sub = rospy.Subscriber("/simu_prompt", String, msg_cb, queue_size=1)

  print('\033[2J')
  rospy.spin()
