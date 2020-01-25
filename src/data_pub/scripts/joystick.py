#!/usr/bin/env python
import rospy
import numpy as np
import XInput as xin
import math
import time
import sys


class Joystick:
    
    NODE_NAME = 'joy_pub'
    LISTENING_TOPIC = 'controls/joystick
    
    def __init__(self):
        self._pub_imu = rospy.Publisher(self.IMU_DEST_TOPIC_QUAT, IMU, queue_size=50)
        

    
if __name__ == '__main__':
try:
    IMURawPublisher().run()
except rospy.ROSInterruptException:
    pass
