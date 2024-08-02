#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Dict, List, Tuple
import rospy
import simpleaudio as sa
from std_msgs.msg import String

def sound_player_cb(msg):
    if not msg.data in sounds:
        rospy.logwarn("Sound not found!")
        return
    sounds[msg.data].play()

if __name__ == "__main__":
    rospy.init_node('sound_player')

    path = "/home/afavier/new_exec_sim_ws/sounds/"

    # load sounds
    sounds = {
        "ns":sa.WaveObject.from_wave_file(          path + "ns.wav"),
        "finished":sa.WaveObject.from_wave_file(    path + "finished_70.wav"),
        
        "yes_box1":sa.WaveObject.from_wave_file(    path + "yes_box1.wav"),
        "box1_full":sa.WaveObject.from_wave_file(   path + "box1_full.wav"),
        "box1_empty":sa.WaveObject.from_wave_file(  path + "box1_empty.wav"),
        
        "yes_box2":sa.WaveObject.from_wave_file(    path + "yes_box2.wav"),
        "box2_full":sa.WaveObject.from_wave_file(   path + "box2_full.wav"),
        "box2_empty":sa.WaveObject.from_wave_file(  path + "box2_empty.wav"),
        
        "yes_box3":sa.WaveObject.from_wave_file(    path + "yes_box3.wav"),
        "box3_full":sa.WaveObject.from_wave_file(   path + "box3_full.wav"),
        "box3_empty":sa.WaveObject.from_wave_file(  path + "box3_empty.wav"),
    }

    sound_sub = rospy.Subscriber('/sound_player', String, sound_player_cb)

    rospy.spin()
