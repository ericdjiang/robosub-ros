#!/usr/bin/env python

import rospy
import roslaunch
from sensor_msgs.msg import Image
import os


class CameraCamera():
    def __init__(self):
        self.leftRecImage = 0
        self.downRecImage = 0
        rospy.init_node("CameraCamera")
        rospy.on_shutdown(self.shutdown)

    def listen(self):
        rospy.Subscriber("/left/image_raw", Image, self.leftImageEvent)
        rospy.Subscriber("/down/image_raw", Image, self.downImageEvent)

    def leftImageEvent(self, msg):
        self.leftRecImage = 1

    def downImageEvent(self, msg):
        self.downRecImage = 1

    def shutdown(self):
        rospy.loginfo("CameraCamera terminated")
        rospy.sleep(1) 

if __name__=="__main__":
    try:
        cam = CameraCamera()
        cam.listen()
        while not rospy.is_shutdown():
            rospy.sleep(5)
            if cam.leftRecImage==0:
                os.system("rosnode kill /left")
                print("left Eric Chang")
                rospy.sleep(15)
            if cam.downRecImage==0:
                os.system("rosnode kill /down")
                print("down Eric Chang")
                rospy.sleep(15)
            cam.leftRecImage=0
            cam.downRecImage=0
    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo("CameraCamera node terinated")
