#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

#######################################
# Our included extra libraries
#######################################
import time
import math
from ardrone_autonomy.msg import Navdata

#######################################
# Our additional values and math to find them
#######################################
H = 6.0 * 0.3 # Convert hight in feet to meters (6ft = 1.8m)
R = 4.0 * 0.3 # Convert radius in feet to meters (4ft = 1.2)
theta = 15.0 # Angle between each pivot
numpoints = 360/theta # Number of pivots
numloops = 3 # Number of loops to cover desired hight
tup = (H/0.2) / numloops # Time that drone increases elevation
tTurn = theta/20.0 # Time to turn (theta / 13.5 for our drone)
tRight = 1 # Time to roll to the right, needs to be set to scale based on radius


# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_E
	PitchBackward    = QtCore.Qt.Key.Key_D
	RollLeft         = QtCore.Qt.Key.Key_S
	RollRight        = QtCore.Qt.Key.Key_F
	YawLeft          = QtCore.Qt.Key.Key_W
	YawRight         = QtCore.Qt.Key.Key_R
	IncreaseAltitude = QtCore.Qt.Key.Key_Q
	DecreaseAltitude = QtCore.Qt.Key.Key_A
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	Emergency        = QtCore.Qt.Key.Key_Space
	# Our additional key
	Orbit     		 = QtCore.Qt.Key.Key_O


# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Emergency:
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff:
				controller.SendTakeoff()
			elif key == KeyMapping.Land:
				controller.SendLand()
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1

				elif key == KeyMapping.PitchForward:
					self.pitch += 1
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1

				elif key == KeyMapping.RollLeft:
					self.roll += 1
				elif key == KeyMapping.RollRight:
					self.roll += -1

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1
				
				# OUr additional Keypress event
				elif key == KEyMapping.Orbit
					self.orbit()
					
			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			
	#############################################
	# Our Additional function for flying in an orbit when 'O' is pressed
	#############################################
	def orbit(self):
		
		# Takeoff and give enough time to settle
		controller.SendTakeoff()
		time.sleep(6)

		# Begin loop that will pivot from i to numLoops
		while ( i < numPoints
			# Turn right
			controller.Setcommand(0, 0, -0.4, 0)
			time.sleep(tTurn)
			controller.Setcommand(0, 0, 0, 0)
			
			# Give it some time to stabalize
			time.sleep(0.5)
			
			# Roll left
			controller.Setcommand(0.5, 0, 0, 0)
			time.sleep(TRight)
			controller.Setcommand(0, 0, 0, 0)
			
			# Give it some time to stabalize
			time.sleep(0.5)
			
			# Incrament the loop
			i += 1
		Controller.SendLand()
	###########################################
	# End of additional code
	###########################################


	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1

			elif key == KeyMapping.PitchForward:
				self.pitch -= 1
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1

			elif key == KeyMapping.RollLeft:
				self.roll -= 1
			elif key == KeyMapping.RollRight:
				self.roll -= -1

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_keyboard_controller')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()

	display.show()

	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
