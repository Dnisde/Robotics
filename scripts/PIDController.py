#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class PIDNode(object):
    def __init__(self):
        rospy.init_node('PID', anonymous=True)
        rospy.Subscriber("goal", Float64, self.getGoal)
        rospy.Subscriber("currentPosition", Float64, self.getCurrentLocation)
        self.pidPub = rospy.Publisher('pidValue', Float64, queue_size = 1)

        self.p = 1.0
        self.i = 0.0
        self.d = 2.0
        # Setting up constants
        self.goal = 0
        self.currentLocation = 0
        self.previousError = 0
        self.integral = 0

        # Rounded PID calculator for 30 times a second
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.runPIDController()
            rate.sleep()

    def runPIDController(self):
        self.p = rospy.get_param("rCtrl/P")
        self.i = rospy.get_param("rCtrl/I")
        self.d = rospy.get_param("rCtrl/D")

        error = self.goal - self.currentLocation

        self.integral = self.integral + error

        if self.integral > 100:
            self.integral = 100
        elif self.integral < -100:
            self.integral = -100

        self.pidPub.publish((self.p*(error) + self.i*(self.integral) + self.d*(error - self.previousError)))
        self.previousError = error
        # print(self.p, self.i, self.d)
        # print("==================================================================")
        # print("previous error is : {}".format(self.previousError))
        # print("Error is : {}".format(error))
        # print("Intergral is : {}".format(self.integral))
        # print("Current Location : {}".format(self.currentLocation))
        # print("Goal is : {}".format(self.goal))
# COMBAK:
    def getGoal(self, goal):
        self.goal = goal.data

    def getCurrentLocation(self, current):
        self.currentLocation = current.data

if __name__ == '__main__':
    node = PIDNode()
