#!/usr/bin/env python
#import rospy
import numpy as np
import XInput as xin
import math
import time
import sys

from geometry_msgs.msg import Twist

#class Joystick:
    
<<<<<<< HEAD
#    NODE_NAME = 'joy_pub'
#    LISTENING_TOPIC = '
    
#    def __init__(self):
#        self._pub_imu = rospy.Publisher(self.IMU_DEST_TOPIC_QUAT, IMU, queue_size=50)
        

    
#if __name__ == '__main__':
#try:
#    IMURawPublisher().run()
#except rospy.ROSInterruptException:
#    pass

print("Hello World!")

#try:
#	while(1):
#		hold = xin.get_thumb_values(xin.get_state(0))
#		print(hold)
#except:
#	exit()

=======
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
>>>>>>> 89eb99d8bfad43cfe60246c74d7e0d9a55f9b2c4
