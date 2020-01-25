#!/usr/bin/env python
import rospy
import numpy as np
import XInput as xin
import math
import time
import sys

from geometry_msgs.msg import Twist

class Joystick:
    
    NODE_NAME = 'joy_pub'
    JOY_DEST_TOPIC_LOCAL = 'controls/desired_twist_local'
    JOY_DEST_TOPIC_GLOBAL = 'controls/desired_twist_global'
    
    def __init__(self):
        self._pub_joy_local = rospy.Publisher(self.JOY_DEST_TOPIC_LOCAL, Twist, queue_size=50)
        self._pub_joy_global = rospy.Publisher(self.JOY_DEST_TOPIC_GLOBAL, Twist, queue_size=50)
        
        self._current_joy_local_msg = Twist()
        self._current_jot_global_msg = Twist()
        
    def run(self):
        rospy.init_node(self.NODE_NAME)

        

    
if __name__ == '__main__':
    try:
        IMURawPublisher().run()
    except rospy.ROSInterruptException:
        pass
