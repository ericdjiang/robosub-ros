#!/usr/bin/env python
import rospy
import numpy as np
import XInput as xin
import math
import time
import sys

try:
    while(1):
        hold = xin.get_thumb_values(xin.get_state(0))

except:
    ser.close()
    exit()
