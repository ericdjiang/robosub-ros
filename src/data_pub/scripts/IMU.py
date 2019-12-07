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
    LINE_DELIM = ','

    def __init__(self):
        self._pub_imu = rospy.Publisher(self.IMU_DEST_TOPIC_QUAT, Imu, queue_size=50)
        self._pub_mag = rospy.Publisher(self.IMU_DEST_TOPIC_MAG, MagneticField, queue_size=50)
        self._pub_imu_rpy = rospy.Publisher(self.IMU_DEST_TOPIC_RPY, Vector3, queue_size=50)
        self._pub_imu_mag = rospy.Publisher(self.IMU_DEST_TOPIC_MAG, Vector3, queue_size=50)
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

        self._serial_port = next(list_ports.grep(self.FTDI_STR)).device
        self._serial = serial.Serial(self._serial_port, self.BAUDRATE, timeout=0.1, write_timeout=1.0,
                                     bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE)

        while not rospy.is_shutdown():
            line = self._serial.readline()
            self._pub_imu.header.stamp = rospy.Time.now()
            items = self._extract_line(line)  # array of line
            self._parse_orient(items)
            self._parse_accel(items)
            self._parse_angvel(items)
            self._publish_current_msg()

    def _extract_line(self, line):
        return line.split(self.LINE_DELIM)

    def _parse_orient(self, items):
        self._current_imu_msg.orientation.w = items[4]
        self._current_imu_msg.orientation.x = items[1]
        self._current_imu_msg.orientation.y = items[2]
        self._current_imu_msg.orientation.z = items[3]

    def _parse_accel(self, items):
        self._current_imu_msg.linear_acceleration.x = items[8]
        self._current_imu_msg.linear_acceleration.y = items[9]
        self._current_imu_msg.linear_acceleration.z = items[10]
        self._current_imu_msg.linear_acceleration_covariance[0] = -1

    def _parse_angvel(self, items):
        self._current_imu_msg.angular_velocity.x = items[11]
        self._current_imu_msg.angular_velocity.y = items[12]
        self._current_imu_msg.angular_velocity.z = items[13]
        self._current_imu_msg.angular_velocity_covariance[0] = -1

    def _publish_current_msg(self):
        self._pub_imu(self._current_imu_msg)
        self._current_imu_msg = Imu()


if __name__ == '__main__':
    try:
        IMURawPublisher.run()
    except rospy.ROSInterruptException:
        pass
