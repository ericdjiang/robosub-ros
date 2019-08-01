#!/usr/bin/env python

# ROS module
import rospy
# Mavlink ROS messages
from mavros_msgs.msg import Mavlink
# pack and unpack functions to deal with the bytearray
from struct import pack, unpack
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import *

# Topic callback
def callback(data):
    # Check if message id is valid (I'm using SCALED_PRESSURE
    # and not SCALED_PRESSURE2)
    if data.msgid == 137:
        rospy.loginfo(rospy.get_caller_id() + " Package: %s", data)
        # Transform the payload in a python string
        p = pack("QQ", *data.payload64)
        # Transform the string in valid values
        # https://docs.python.org/2/library/struct.html
        time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)
        pub=rospy.Publisher('pressure',FluidPressure,queue_size=10)
        ret=FluidPressure()
        ret.fluid_pressure=press_abs
        ret.variance=0
        pub.publish(ret)
    
def listener():
    
    rospy.init_node('pixhawk_pressure', anonymous=True)
    rospy.Subscriber("/mavlink/from", Mavlink, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()


