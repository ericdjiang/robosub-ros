#!/usr/bin/env python
import rospy
import numpy as np
import math
import serial
import time

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import euler_from_quaternion

IMU_DEST_TOPIC_RPY = 'sensors/imu/rpy'
IMU_DEST_TOPIC_MAG = 'sensors/imu/mag'
IMU_DEST_TOPIC_ACCEL = 'sensors/imu/accel'
IMU_DEST_TOPIC_GYRO = 'sensors/imu/gyro'
IMU_DEST_TOPIC_QUAT = 'sensors/imu/quat'
NODE_NAME = 'imu_pub'

ser = serial.Serial("COM11", 115200)
ser.close()
ser.open()

def talker():
    rospy.init_node(NODE_NAME)
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

    try:
        while(1):
            line = str(ser.readline())
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

            imu_pub_data.publish(imu_msg)

            mag_msg.magnetic_field.x = items[5]
            mag_msg.magnetic_field.y = items[6]
            mag_msg.magnetic_field.y = items[7]

            #print("yaw:\t" + str(float(items[1])) + "\t\t pitch: \t" + str(float(items[2])) + "\t roll: \t" + str(float(items[3])))
            #time.sleep(0.5)
    except:
        ser.close()
        exit()

if __name__ == '__main__':
    talker()