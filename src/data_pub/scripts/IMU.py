#!/usr/bin/env python
import rospy
import numpy as np
import math
import serial
import time
import serial.tools.list_ports as list_ports
import sys

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import euler_from_quaternion


class IMURawPublisher:
    IMU_DEST_TOPIC_RPY = 'sensors/imu/rpy'
    IMU_DEST_TOPIC_MAG = 'sensors/imu/mag'
    IMU_DEST_TOPIC_ACCEL = 'sensors/imu/accel'
    IMU_DEST_TOPIC_GYRO = 'sensors/imu/gyro'
    IMU_DEST_TOPIC_QUAT = 'sensors/imu/quat'

    FTDI_STR = 'ttyUSB0'
    BAUDRATE = 115200
    NODE_NAME = 'imu_pub'
    LINE_DELIM = ','

    def __init__(self):
        self._pub_imu = rospy.Publisher(self.IMU_DEST_TOPIC_QUAT, Imu, queue_size=50)
        self._pub_imu_rpy = rospy.Publisher(self.IMU_DEST_TOPIC_RPY, Vector3, queue_size=50)
        self._pub_imu_mag = rospy.Publisher(self.IMU_DEST_TOPIC_MAG, MagneticField, queue_size=50)
        self._pub_imu_accel = rospy.Publisher(self.IMU_DEST_TOPIC_ACCEL, Vector3, queue_size=50)
        self._pub_imu_gyro = rospy.Publisher(self.IMU_DEST_TOPIC_GYRO, Vector3, queue_size=50)

        self._current_imu_msg = Imu()
        self._current_mag_msg = MagneticField()
        self._current_imu_msg.header.frame_id = "imu_link"
        self._current_mag_msg.header.frame_id = "mag_link"

        self._serial_port = None
        self._serial = None

    def run(self):
        rospy.init_node(self.NODE_NAME)
#        ports = list_ports.comports()
#        for i in ports:
#	   print(i.name)
        self._serial_port = next(list_ports.grep(self.FTDI_STR)).device
        self._serial = serial.Serial(self._serial_port, self.BAUDRATE, timeout=None, write_timeout=None, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        
        while not rospy.is_shutdown():
	    
            #while(1):
	    #    line = self._serial.read_until()
	#	print(line)
	   # for i in range(1):
	#	line = self._serial.read_until()
	    line = self._serial.read_until()	
           # rospy.loginfo(line)
            #self._pub_imu.header.stamp = rospy.Time.now()
            items = self._extract_line(line)  # array of line
	    if items[0] == "$VNQMR":
	        rospy.loginfo(line)
                self._parse_orient(items)
                self._parse_accel(items)
                self._parse_angvel(items)
	        self._parse_mag(items)
                self._publish_current_msg()

    def _extract_line(self, line):
        return line.split(self.LINE_DELIM)

    def _parse_orient(self, items):
        self._current_imu_msg.orientation.x = float(items[1])
        self._current_imu_msg.orientation.y = float(items[2])
        self._current_imu_msg.orientation.z = float(items[3])
        self._current_imu_msg.orientation.w = float(items[4])
	self._current_imu_msg.orientation_covariance[0] = -1

    def _parse_accel(self, items):
        self._current_imu_msg.linear_acceleration.x = float(items[8])
        self._current_imu_msg.linear_acceleration.y = float(items[9])
        self._current_imu_msg.linear_acceleration.z = float(items[10])
        self._current_imu_msg.linear_acceleration_covariance[0] = -1

    def _parse_angvel(self, items):
        self._current_imu_msg.angular_velocity.x = float(items[11])
        self._current_imu_msg.angular_velocity.y = float(items[12])
	items[13] = items[13][0:10]
        self._current_imu_msg.angular_velocity.z = float(items[13])
        self._current_imu_msg.angular_velocity_covariance[0] = -1

    def _parse_mag(self, items):
	self._current_mag_msg.magnetic_field.x = float(items[5])
	self._current_mag_msg.magnetic_field.y = float(items[6])
	self._current_mag_msg.magnetic_field.z = float(items[7])
	self._current_mag_msg.magnetic_field_covariance[0] = -1

    def _publish_current_msg(self):
        self._pub_imu.publish(self._current_imu_msg)
        self._current_imu_msg = Imu()
	self._pub_imu_mag.publish(self._current_mag_msg)
	self._current_mag_msg = MagneticField()

if __name__ == '__main__':
    try:
        IMURawPublisher().run()
    except rospy.ROSInterruptException:
        pass
