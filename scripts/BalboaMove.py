#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaMotorSpeeds
from geometry_msgs.msg import Twist
import sys, select, termios, tty, time

class BalboaMoveControls:

    def __init__ (self):
        self.pub = rospy.Publisher("motorSpeeds",balboaMotorSpeeds, queue_size = 5)
        rospy.init_node('BalboaMove')
        self.sub = rospy.Subscriber('/turtle1/cmd_vel', Twist, self.handleMessage, queue_size = 1)
        rospy.spin()


    def handleMessage(self, twist):
        print(twist)
        motorSpeeds = balboaMotorSpeeds()
        motorSpeeds.left = 0
        motorSpeeds.right = 0
        # rate = rospy.Rate(10);  # go through the loop 10 times per second
        rospy.loginfo("Start Running")
        if (twist.linear.x != 0):
            if (twist.linear.x > 0):
                motorSpeeds.left = 15
                motorSpeeds.right = 15
            if (twist.linear.x < 0):
                motorSpeeds.left = -15
                motorSpeeds.right = -15
        if (twist.angular.z != 0):
            if (twist.angular.z > 0):
                motorSpeeds.right += 7.5
            if (twist.angular.z < 0):
                motorSpeeds.left += 7.5
        self.pub.publish(motorSpeeds)
        time.sleep(0.1) # for how much it travel before reseting speed stopping
        motorSpeeds.left = 0
        motorSpeeds.right = 0
        self.pub.publish(motorSpeeds)


if __name__ == '__main__':
    moveNode = BalboaMoveControls()
