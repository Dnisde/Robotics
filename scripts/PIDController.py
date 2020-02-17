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

        self.goal = 0
        self.currentLocation = 0
        self.previousError = 0
        self.integral = 0

        rate = rospy.Rate(30) #30Hz
        while not rospy.is_shutdown():
            self.runPIDController()
            rate.sleep()

    def runPIDController(self):
        self.p = rospy.get_param("rCtrl/P")
        self.i = rospy.get_param("rCtrl/I")
        self.d = rospy.get_param("rCtrl/D")

        error = self.goal - self.currentLocation

        self.integral = self.integral + error
        if self.integral > 50:
            self.integral = 50
        elif self.integral < -50:
            self.integral = -50
            
        self.pidPub.publish((self.p*(error) + self.i*(self.integral) + self.d*(error - self.previousError)))
        self.previousError = error

    def getGoal(self, goal):
        self.goal = goal.data

    def getCurrentLocation(self, current):
        self.current = current.data

if __name__ == '__main__':
    node = PIDNode()
