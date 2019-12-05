#!/usr/bin/env python
import rospy
import numpy as np
import math
import serial
import time
import serial.tools.list_ports as list_ports

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import euler_from_quaternion

class IMURawPublisher:

    IMU_DEST_TOPIC_RPY = 'sensors/imu/rpy'
    IMU_DEST_TOPIC_MAG = 'sensors/imu/mag'
    IMU_DEST_TOPIC_ACCEL = 'sensors/imu/accel'
    IMU_DEST_TOPIC_GYRO = 'sensors/imu/gyro'
    IMU_DEST_TOPIC_QUAT = 'sensors/imu/quat'

    FTDI_STR = 'FT232R'
    BAUDRATE = 115200
    NODE_NAME = 'imu_pub'

    def __init__(self):
        self._pub_imu = rospy.Publisher(self.IMU_DEST_TOPIC_QUAT, Imu, queue_size=50)
        self._pub_mag = rospy.Publisher(self.IMU_DEST_TOPIC_MAG, MagneticField, queue_size=50)

        self._current_imu_msg = Imu()
        self._current_mag_msg = MagneticField()

        self._serial_port = None
        self._serial = None

    def run(self):
        rospy.init_node(self.NODE_NAME)

        self._serial_port = next(list_ports.grep(FTDI_STR)).device
        self._serial = serial.Serial(_serial_port, BAUDRATE, timeout=0.1, write_timeout=1.0,
                                     bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE)

        while not rospy.is_shutdown():



def talker():
    imu_pub_rpy = rospy.Publisher(IMU_DEST_TOPIC_RPY, Vector3, queue_size=50)
    imu_pub_mag = rospy.Publisher(IMU_DEST_TOPIC_MAG, Vector3, queue_size=50)
    imu_pub_accel = rospy.Publisher(IMU_DEST_TOPIC_ACCEL, Vector3, queue_size=50)
    imu_pub_gyro = rospy.Publisher(IMU_DEST_TOPIC_GYRO, Vector3, queue_size=50)

    imu_pub_data = rospy.Publisher(IMU_DEST_TOPIC_QUAT, Imu, queue_size=50)
    mag_pub_data = rospy.Publisher(IMU_DEST_TOPIC_MAG, MagneticField, queue_size=50)
    imu_msg = Imu()
    mag_msg = MagneticField()
    imu_msg.header.frame_id = "imu_link"
    mag_msg.header.frame_id = "mag_link"

    _serial_port = next(list_ports.grep(FTDI_STR)).device
    _serial = serial.Serial(_serial_port, BAUDRATE, timeout=0.1, write_timeout=1.0,
                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE)

    try:
        line = str(self._serial.readline())
        items = line.split(",")
        imu_pub_data.header.stamp = rospy.Time.now()
        imu_pub_data.header.stamp = rospy.Time.now()

        imu_msg.orientation.w = items[4]
        imu_msg.orientation.x = items[1]
        imu_msg.orientation.y = items[2]
        imu_msg.orientation.z = items[3]

        imu_msg.linear_acceleration.x = items[8]
        imu_msg.linear_acceleration.y = items[9]
        imu_msg.linear_acceleration.z = items[10]
        imu_msg.linear_acceleration_covariance[0] = -1

        imu_msg.angular_velocity.x = items[11]
        imu_msg.angular_velocity.y = items[12]
        imu_msg.angular_velocity.z = items[13]
        imu_msg.angular_velocity_covariance[0] = -1
if __name__ == '__main__':
    try:
        IMURawPublisher.run()
    except rospy.ROSInterruptException:
        pass
