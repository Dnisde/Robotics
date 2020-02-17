#!/usr/bin/env python
import rospy, math, time
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class distanceTargetControls:

    def __init__ (self):
        self._start_time = 0 # initial start time with key pressing
        self.time_space = 0.02 # The time interval between everytime you pressed the key?
        self.time1 = 0.0
        self.time2 = 0.0
        balboa = 'motorSpeeds'
        rospy.init_node('distancePID', anonymous = True) # Node name: distancePID
        self.publisher = rospy.Publisher(balboa, balboaMotorSpeeds, queue_size = 10)

        # publish to:
        # topic's name: motorSpeeds,
        # message/type: balboaMotorSpeeds: int8 left, int8 right

    def transferMessage(self, twist):
        print(twist) # Start recieve signal from Keyboard?
        motorSpeeds = balboaMotorSpeeds()
        motorSpeeds.left = 0
        motorSpeeds.right = 0
        # rate = rospy.Rate(100) # go through the loop 100 times per second
        linear_time = self._start_time # initialize time
        if(twist.linear.x != 0): # Start pressin the arrow keys?

            # Pressing the UP arrow
            self.time1 = time.clock() # Start recording the time
            if(twist.linear.x > 0):
                twist.linear.x = 0
                time.sleep(self.time_space) # sleep for 1/50 seconds
                self.publisher.publish(motorSpeeds)
                # TODO with adjusting distance

            # Pressing the DOWN arrow
            self.time1 = time.clock() # Start recording the time
            if(twist.linear.x < 0):
                twist.linear.x = 0
                time.sleep(self.time_space) # sleep for 1/50 seconds
                self.publisher.publish(motorSpeeds)
                # TODO with adjusting distance

            self.time1 = 0.0 # initialize time variable
            self.time2 = 0.0
        print('-----------------------------')
        print(twist) # Start recieve signal from Keyboard?
        print('-----------------------------')
        print('-----------------------------')

    def timmer(self):
        keyboard = '/turtle1/cmd_vel'
        rospy.Subscriber(keyboard, Twist, self.transferMessage)
        # subsribe from:
        # topic's name: Keyboard: turtle_teleop_key. Arrow keys command.
        # message/type: Twist:
        # geometry_msgs/Vector3 linear: float64 x, float64 y, float64 z
        # geometry_msgs/Vector3 angular: float64 x, float64 y, float64 z
        # message, handle function: transferMessage
        self.time2 = time.clock()
        linear_time = self.time2 - self.time1
        print "time elapse : {}".format(linear_time)
        linear_time = 0.0 # initialize time variable
        rospy.spin()


if __name__ == '__main__':
    distanceNode = distanceTargetControls()
    distanceNode.timmer()
