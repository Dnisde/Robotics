#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from balboa_core.msg import balboaLL

class RangeFinder(object):
    def __init__(self):
        rospy.init_node('RangeFinder', anonymous=True)
        self.sensorSub = rospy.Subscriber("/balboaLL", balboaLL, self.handleRange, queue_size = 1)
        rospy.Subscriber("currentLinear", Float64, self.getCurrentLocation)
        self.goalPub = rospy.Publisher('linearGoal', Float64, queue_size = 1)
        self.goalRange = 50 #default value
        self.currentLocation = 0
        rospy.spin()

    def handleRange(self, balboaLLMessage):
        rangeValue = balboaLLMessage.range
        if(rangeValue > 150):
            rangeValue = 150
        elif(rangeValue < 15):
            rangeValue = 15
        error = self.goalRange - rangeValue
        print("Range Value is: {}".format(rangeValue))
        print("\n")
        print("Error is: {}".format(error))
        print("\n")
        print("Current Location is: {}".format(self.currentLocation))
        if(error > 2 or error < -2):
            self.goalPub.publish((self.currentLocation + (error/2.54)))

    def getCurrentLocation(self, current):
        self.currentLocation = current.data




if __name__ == '__main__':
    node = RangeFinder()
