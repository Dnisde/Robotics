#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaMotorSpeeds
from std_msgs.msg import String, Float64
import time

class BalboaMoveControls:

	def __init__ (self):
		self.motorPub = rospy.Publisher("motorSpeeds",balboaMotorSpeeds, queue_size = 5)
		self.turnPub = rospy.Publisher("turnSignal",Float64, queue_size = 5)
		rospy.init_node('BalboaMove')
		self.movement = True
		self.sub = rospy.Subscriber('movementCommand', Float64, self.setMovement, queue_size = 1)        
		self.motorSpeeds = balboaMotorSpeeds()
		self.motorSpeeds.left = 0
		self.motorSpeeds.right = 0
		self.turnDirection = True #true for left, false for right
		rospy.spin()

	def setMovement(self, movement):
		if (movement == "go"):
			self.movement = True
			patternMovement()
		elif (movement == "stop"):
			self.movement = False
			
	def patternMovement(self):
		while (self.movement):
			goStraight()
			time.sleep(1)
			turn()
			time.sleep(1)
		if (not self.movement):
			self.motorSpeeds.left = 0
			self.motorSpeeds.right = 0
			self.motorPub.publish(self.motorSpeeds)
			
	def goStraight(self):
		self.turnPub.publish(0)
		self.motorSpeeds.left = 10
		self.motorSpeeds.right = 10
		self.motorPub.publish(self.motorSpeeds)

	def turn(self):
		if (self.turnDirection):
			self.motorSpeeds.left = 0
			self.motorSpeeds.right = 5
			self.motorPub.publish(self.motorSpeeds)
			self.turnPub.publish(0)
			self.turnDirection = false
		elif (not self.turnDirection):
			self.motorSpeeds.left = 5
			self.motorSpeeds.right = 0
			self.motorPub.publish(self.motorSpeeds)
			self.turnPub.publish(1)
			self.turnDirection = True
if __name__ == '__main__':
    moveNode = BalboaMoveControls()