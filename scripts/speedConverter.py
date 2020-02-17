#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaMotorSpeeds
from std_msgs.msg import Float64

MOTOR_SPEED_CAP = 20
LINEAR_SPEED_SCALE = 0.0
ANGULAR_SPEED_SCALE = 0.25

class SpeedConverterNode(object):
    def __init__(self):
        rospy.init_node('speedConverter', anonymous=True)
        rospy.Subscriber("angularOutput", Float64, self.getAngularPIDOutputAndCalculateSpeed)
        rospy.Subscriber("linearOutput", Float64, self.getLinearPIDOutputAndCalculateSpeed)
        self.motorSpeedPublisher = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size = 1)

        self.angularPIDValue = 0
        self.linearPIDValue = 0

        rospy.spin()

    def getAngularPIDOutputAndCalculateSpeed(self, output):
        self.angularPIDValue = output.data

        motorSpeed = balboaMotorSpeeds()

        motorSpeed.left = self.linearPIDValue*.25
        motorSpeed.right = self.linearPIDValue*.25

        motorSpeed.left = motorSpeed.left - self.angularPIDValue*.25
        motorSpeed.right = motorSpeed.right + self.angularPIDValue*.25

        if(motorSpeed.left > 25):
            motorSpeed.left = 25
        elif(motorSpeed.left < -25):
            motorSpeed.left = -25

        if(motorSpeed.right > 25):
            motorSpeed.right = 25
        elif(motorSpeed.right < -25):
            motorSpeed.right = -25

        self.motorSpeedPublisher.publish(motorSpeed)


    def getLinearPIDOutputAndCalculateSpeed(self, output):
        self.linearPIDValue = output.data
        
        motorSpeed = balboaMotorSpeeds()

        motorSpeed.left = self.linearPIDValue*.25
        motorSpeed.right = self.linearPIDValue*.25

        motorSpeed.left = motorSpeed.left - self.angularPIDValue*.25
        motorSpeed.right = motorSpeed.right + self.angularPIDValue*.25

        if(motorSpeed.left > 25):
            motorSpeed.left = 25
        elif(motorSpeed.left < -25):
            motorSpeed.left = -25

        if(motorSpeed.right > 25):
            motorSpeed.right = 25
        elif(motorSpeed.right < -25):
            motorSpeed.right = -25

        self.motorSpeedPublisher.publish(motorSpeed)

if __name__ == '__main__':
    node = SpeedConverterNode()
