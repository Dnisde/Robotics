#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaLL
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class InputConverterNode(object):
    def __init__(self):
        rospy.init_node( 'inputConverter', anonymous=True )
        rospy.Subscriber("/balboaLL", balboaLL, self.convertUnits)
        rospy.Subscriber("/turtle1/cmd_vel", Twist, self.createGoal)

        self.currentAngular = 0
        self.currentLinear = 0
        self.angularGoal = self.currentAngular
        self.linearGoal = self.currentLinear

        self.anglePub = rospy.Publisher("angularGoal", Float64, queue_size = 1)
        self.distancePub = rospy.Publisher("linearGoal", Float64, queue_size = 1)
        self.currentAnglePub = rospy.Publisher("currentAngular", Float64, queue_size = 1)
        self.currentDistancePub = rospy.Publisher("currentLinear", Float64, queue_size = 1)

        rospy.spin()

    def convertUnits(self, balboaLLMessage):
        leftLinear  = balboaLLMessage.distanceLeft
        rightLinear = balboaLLMessage.distanceRight

        self.currentAngular = balboaLLMessage.angleX / 970.00
        self.currentLinear = self.calculateDistance(leftLinear, rightLinear) / 130.0
        self.currentAnglePub.publish(Float64(self.currentAngular))
        self.currentDistancePub.publish(Float64(self.currentLinear))

    def calculateDistance(self, left, right):
        return (left + right)/2.0

    def createGoal(self, twist):
        self.linearGoal = self.currentLinear
        self.angularGoal = self.currentAngular

        self.angularGoal = self.angularGoal + (twist.angular.z*10)
        self.linearGoal = self.linearGoal + (twist.linear.x*4)
        print("==================================================================")
        print("Our angular goal is :{}".format(self.angularGoal))
        print("Current angular is :{}".format(self.currentAngular))
        print("Our distance goal is :{}".format(self.linearGoal))
        print("Current distance is :{}".format(self.currentLinear))
        self.anglePub.publish(Float64(self.angularGoal))
        self.distancePub.publish(Float64(self.linearGoal))


if __name__ == '__main__':
    node = InputConverterNode()
