#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import glob

i=0

def image_callback(msg):
    global i
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        filename = f"/tmp/save_frames_from_simulator/frame_{i}.jpg"
        cv2.imwrite(filename, cv_image)
        # rospy.loginfo(f"Saved frame at {filename}")
        i+=1
    except CvBridgeError as e:
        rospy.logerr(f"Could not convert image: {e}")

if __name__ == '__main__':

    try:
        os.mkdir("/tmp/save_frames_from_simulator")
    except:
        pass

    try:
        for f in glob.glob("/tmp/save_frames_from_simulator/*"):
            os.remove(f)
    except:
        pass

    rospy.init_node('camera_listener', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber("/bird_camera/rgb/image_raw", Image, image_callback, queue_size=1)
    rospy.spin()
