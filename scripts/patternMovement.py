#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaMotorSpeeds
from geometry_msgs.msg import Twist
import sys, select, termios, tty, time

class BalboaMoveControls:

    def __init__ (self):
        self.pub = rospy.Publisher("motorSpeeds",balboaMotorSpeeds, queue_size = 5)
        rospy.init_node('BalboaMove')
		self.movement = true
        self.sub = rospy.Subscriber('movementCommand', String, self.setMovement, queue_size = 1)        
		self.motorSpeeds = balboaMotorSpeeds()
        self.motorSpeeds.left = 0
        self.motorSpeeds.right = 0
		self.turnDirection = true #true for left, false for right
        rospy.spin()

	def setMovement(self, movement):
		if (movement == "go"):
			self.movement = true
			patternMovement()
		elif (movement == "stop"):
			self.movement = false
			
	def patternMovement(self):
		while (self.movement):
			goStraight()
			time.sleep(1)
			turn()
			time.sleep(1)
		if (!self.movement):
			self.motorSpeeds.left = 0
			self.motorSpeeds.right = 0
			self.pub.publish(self.motorSpeeds)
			
	def goStraight(self):
		self.motorSpeeds.left = 10
		self.motorSpeeds.right = 10
		self.pub.publish(self.motorSpeeds)

	def turn(self):
		if (turnDirection):
			self.motorSpeeds.left = 0
			self.motorSpeeds.right = 5
			self.pub.publish(self.motorSpeeds)
			self.turnDirection = false
		elif (!turnDirection):
			self.motorSpeeds.left = 5
			self.motorSpeeds.right = 0
			self.pub.publish(self.motorSpeeds)
			self.turnDirection = true
if __name__ == '__main__':
    moveNode = BalboaMoveControls()