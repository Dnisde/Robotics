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

        self.currentAngular = 0;
        self.currentLinear = 0;

        self.anglePub = rospy.Publisher("angularGoal", Float64, queue_size = 1)
        self.distancePub = rospy.Publisher("distanceGoal", Float64, queue_size = 1)
        self.currentAnglePub = rospy.Publisher("currentAngular", Float64, queue_size = 1)
        self.currentDistancePub = rospy.Publisher("currentLinear", Float64, queue_size = 1)

        rospy.spin()

    def convertUnits(self, balboaLLMessage):
        leftLinear  = balboaLLMessage.distanceLeft
        rightLinear = balboaLLMessage.distanceRight

        self.currentAngular = balboaLLMessage.angleX / 970.00
        self.currentLinear = calculateDistance(leftLinear, rightLinear) / 120
        self.currentAnglePub.publish(Float64(self.currentAngular))
        self.currentDistancePub.publish(Float64(self.currentLinear))

    def calculateDistance(self, left, right):
        return (left + right)/2

    def createGoal(self, twist):
        linearGoal = self.currentLinear
        angularGoal = self.currentAngular

        angularGoal = angularGoal + (twist.angular.z *.5)
        linearGoal = linearGoal + (twist.linear.x *.1)

        self.anglePub.publish(Float64(angularGoal))
        self.distancePub.publish(Float64(distanceGoal))


if __name__ == '__main__':
    node = InputConverterNode()
