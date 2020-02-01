############ Logitech F310 Gamepad Controller Main - parser_main.py ##########
# Original Author: John Zeller
# Description: Parser_Main polls the data in a python dictionary to check the
#			   states of several buttons coming from parser_core.py. Once it
#			   reads these states, it will display them for reference on the
#			   terminal it was launched in

### NOTES #####################################################################
# 1) LEAVE MODE 'OFF' there is no support in parser_main.py for MODE ON
# 2) Naturally the gamepad sends the following values:
# 	LJ/RJ - Down: 0-127
# 	LJ/RJ - Up: 255-128
# 	LJ/RJ - Left: 255-128
# 	LJ/RJ - Right: 0-127
###############################################################################

### Buttons/Joys Represented: #################################################
# A, B, X, Y, RB, LB, LJButton, RJButton, Back, Start, Middle
# RT, LT, LeftJoy, RightJoy, Left, Right, Up, Down
###############################################################################

import sys
sys.path.append("./core")
import threading
from bus import *
from parser_core import *
from gui_main import *
import Tkinter as tk	# Native Python GUI Framework
import time

import rospy
import numpy as np
import XInput as xin
import math
import time
import sys

from geometry_msgs.msg import Twist

class ParserMain(threading.Thread):
	
	NODE_NAME = 'joy_pub'
	JOY_DEST_TOPIC_LOCAL = 'controls/desired_twist_local'
	JOY_DEST_TOPIC_GLOBAL = 'controls/desired_twist_global'

	def __init__(self):
		
		self._pub_joy_local = rospy.Publisher(self.JOY_DEST_TOPIC_LOCAL, Twist, queue_size=50)
        self._pub_joy_global = rospy.Publisher(self.JOY_DEST_TOPIC_GLOBAL, Twist, queue_size=50)
        
        self._current_joy_local_msg = Twist()
        self._current_jot_global_msg = Twist()
		
		self._movement_type = 0		# Where 0 defines the left joystick to translational movement (x, y)
									#   and 1 defines the left joystick to rotation (roll, pitch)
			
		self._global_state = 0		# Where 0 is local movement
									#   and 1 is global movement
		
		# Create bus object
		self.bus = Bus()
		# Create a dictionary to be used to keep states from joy_core
		self.states = { 'A':0, 'B':0, 'X':0, 'Y':0,  		   		\
				'Back':0, 'Start':0, 'Middle':0,		   			\
				'Left':0, 'Right':0, 'Up':0, 'Down':0, 				\
				'LB':0, 'RB':0, 'LT':0, 'RT':0,						\
				'LJ/Button':0, 'RJ/Button':0, 						\
				'LJ/Left':0, 'LJ/Right':0, 'LJ/Up':0, 'LJ/Down':0, 	\
				'RJ/Left':0, 'RJ/Right':0, 'RJ/Up':0, 'RJ/Down':0}	

                '''
		'Byte0':0, 'Byte1':0, 'Byte2':0, 'Byte3':0,			
		'Byte4':0, 'Byte5':0, 'Byte6':0, 'Byte7':0,			
		'Byte0/INT':0, 'Byte1/INT':0, 'Byte2/INT':0, 		
		'Byte3/INT':0, 'Byte4/INT':0, 'Byte5/INT':0, 		
		'Byte6/INT':0, 'Byte7/INT':0}
                '''

		# Launch Parser_Core as a seperate thread to parse the gamepad
		self.parsercore = ParserCore(self.bus, self.states)
		self.parsercore.start()
	
	def run(self):
		
		rospy.init_node(self.NODE_NAME)
		
	   	while(1):
			leftLR = self.states['LJ/Left' + 'LJ/Right']
			leftUD = self.states['LJ/Up' + 'LJ/Down']
			rightLR = self.states['RJ/Left' + 'RJ/Right']
			rightUD = self.states['RJ/Up' + 'RJ/Down']

			if (self.states['X'] == 1):
				self._movement_type = 0

			if (self.states['Y'] == 1):
				self._movement_type = 1
				
			if (self.states['A'] == 1):
				self._global_state = 0
				
			if (self.states['B'] == 1):
				self._global_state = 1
				
			if (self._global_state == 0):
				if (self._movement_type == 0): 
					self._current_joy_local_msg.linear.x = leftLR
					self._current_joy_local_msg.linear.y = leftUD
					self._current_joy_local_msg.linear.z = rightUD

					self._current_joy_local_msg.angular.x = 0			
					self._current_joy_local_msg.angular.y = 0			
					self._current_joy_local_msg.angular.z = rightLR

				else:
					self._current_joy_local_msg.linear.x = 0			
					self._current_joy_local_msg.linear.y = 0
					self._current_joy_local_msg.linear.z = rightUD

					self._current_joy_local_msg.angular.x = leftLR
					self._current_joy_local_msg.angular.y = leftUD				
					self._current_joy_local_msg.angular.z = rightLR
			
			if (self._global_state == 1):
				if (self._movement_type == 0): 
					self._current_joy_global_msg.linear.x = leftLR
					self._current_joy_global_msg.linear.y = leftUD
					self._current_joy_global_msg.linear.z = rightUD

					self._current_joy_global_msg.angular.x = 0			
					self._current_joy_global_msg.angular.y = 0			
					self._current_joy_global_msg.angular.z = rightLR

				else:
					self._current_joy_global_msg.linear.x = 0			
					self._current_joy_global_msg.linear.y = 0
					self._current_joy_global_msg.linear.z = rightUD

					self._current_joy_global_msg.angular.x = leftLR
					self._current_joy_global_msg.angular.y = leftUD				
					self._current_joy_global_msg.angular.z = rightLR
					
			self._pub_joy_local.publish(self._current_joy_local_msg)
			self._current_joy_local_msg = Twist()
			self._pub_joy_global.publish(self._current_joy_global_msg)
			self._current_joy_global_msg = Twist()
				
			
				

		
if __name__ == '__main__':
	try:
        	parser = ParserMain()
        	parser.run()
	except rospy.ROSInterruptException:
        	pass
