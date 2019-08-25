#!/usr/bin/env python

import serial
from serial.tools import list_ports
import rospy
import numpy as np
import math

from std_msgs.msg import String
from navigator_dvl.msg import DVLRaw
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class DvlRawPublisher:

    FTDI_STR = 'FT232R'
    BAUDRATE = 115200
    DVL_RAW_TOPIC = 'sensors/dvl/raw'
    DVL_ODOM_TOPIC = 'sensors/dvl/odom'
    NODE_NAME = 'dvl_raw_publisher'
    LINE_DELIM = ','
    DVL_BAD_STATUS_MSG = 'V'

    def __init__(self):
        self._raw_pub = rospy.Publisher(self.DVL_RAW_TOPIC, DVLRaw, queue_size=10)
        self._odom_pub = rospy.Publisher(self.DVL_ODOM_TOPIC, Odometry, queue_size=10)
        self._current_raw = DVLRaw()

        self._serial_port = None
        self._serial = None

        self._dvl_line_parsers = {
            'SA': self._parse_SA,
            'TS': self._parse_TS,
            'BI': self._parse_BI,
            'BS': self._parse_BS,
            'BE': self._parse_BE,
            'BD': self._parse_BD
        }


    def run(self):
        rospy.init_node(self.NODE_NAME)

        self._serial_port = next(list_ports.grep(self.FTDI_STR)).device
        self._serial = serial.Serial(self._serial_port, self.BAUDRATE, 
                timeout=0.1, write_timeout=1.0,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE)

        while not rospy.is_shutdown():
            line = self._serial.readline()
            if line[0] == ':':
                self._parse_line(line)

    def _parse_line(self, line):
        data_type = line[1:3]
        self._dvl_line_parsers.get(data_type, self._parse_skip)(self._clean_line(line))

    def _clean_line(self, line):
        return line[4:].replace('\r\n', '')

    def _parse_SA(self, line):
        fields = self._extract_floats(line, 0, None)
        self._current_raw.sa_roll = fields[0]
        self._current_raw.sa_pitch = fields[1]
        self._current_raw.sa_heading = fields[2]

    def _parse_TS(self, line):
        fields = self._extract_floats(line, 1, None)
        self._current_raw.ts_salinity = fields[0]
        self._current_raw.ts_temperature = fields[1]
        self._current_raw.ts_depth = fields[2]
        self._current_raw.ts_sound_speed = fields[3]
        self._current_raw.ts_built_in_test = int(fields[4])

    def _parse_BI(self, line):
        fields = self._extract_floats(line, 0, 4)
        self._current_raw.bi_x_axis = fields[0]
        self._current_raw.bi_y_axis = fields[1]
        self._current_raw.bi_z_axis = fields[2]
        self._current_raw.bi_error = fields[3]
        self._current_raw.bi_status = line.split(self.LINE_DELIM)[4]

    def _parse_BS(self, line):
        fields = self._extract_floats(line, 0, 3)
        self._current_raw.bs_transverse = fields[0]
        self._current_raw.bs_longitudinal = fields[1]
        self._current_raw.bs_normal = fields[2]
        self._current_raw.bs_status = line.split(self.LINE_DELIM)[3]

    def _parse_BE(self, line):
        fields = self._extract_floats(line, 0, 3)
        self._current_raw.be_east = fields[0]
        self._current_raw.be_north = fields[1]
        self._current_raw.be_upwards = fields[2]
        self._current_raw.be_status = line.split(self.LINE_DELIM)[3]

    def _parse_BD(self, line):
        fields = self._extract_floats(line, 0, None)
        self._current_raw.bd_east = fields[0]
        self._current_raw.bd_north = fields[1]
        self._current_raw.bd_upwards = fields[2]
        self._current_raw.bd_range = fields[3]
        self._current_raw.bd_time = fields[4]

        # BD type is the last message received, so publish
        self._publish_current_raw()
    
    def _parse_skip(self, line):
        pass

    def _extract_floats(self, num_string, start, stop):
        """Return a list of floats from a given string,
        using LINE_DELIM and going from start to stop
        """
        return [float(num) 
            for num 
            in num_string.split(self.LINE_DELIM)[start:stop]]
    
    def _publish_current_raw(self):
        """Publish the current DVL message and set the message to empty
        """
        current_time = rospy.Time.now()
        self._current_raw.header.stamp = current_time
        self._raw_pub.publish(self._current_raw)
        if self._current_raw.bs_status != DVL_BAD_STATUS_MSG:
            # check if the data is good (for now, only check bs and sa status as they are the only two data that we are currently using) (there is no status for sa)
            # for status: A = good, V = bad
            self._odom_pub.publish(self._current_odom())
        self._current_raw = DVLRaw()

    def _current_odom(self):
        # handle message here
        odom = Odometry()
        odom.header.stamp = self._current_raw.header.stamp
        odom.header.frame_id = 'odom'

        # bs velocity, normalized to meters (given in mm)
        vx = -np.float64(self._current_raw.bs_longitudinal) / 1000
        vy = np.float64(self._current_raw.bs_transverse) / 1000
        vz = np.float64(self._current_raw.bs_normal) / 1000

        # quat
        roll = math.radians(np.float64(self._current_raw.sa_roll))
        pitch = math.radians(np.float64(self._current_raw.sa_pitch))
        yaw = math.radians(np.float64(self._current_raw.sa_heading))
        odom_quat = quaternion_from_euler(roll, pitch, yaw)

        # set pose, (set position to (0, 0, 0), should not be used)
        odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(*odom_quat))
        odom.child_frame_id = "dvl_link"
        # set odom (set angular velocity to (0, 0, 0), should not be used)
        odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
        return odom

if __name__ == '__main__':
    try:
    	DvlRawPublisher().run()
    except rospy.ROSInterruptException:
    	pass
