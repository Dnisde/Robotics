#!/usr/bin/env python
import rospy
from lab2.msg import balboaLLL
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class readFromSensorNode(object):

    def __init__(self):
        rospy.init_node('ReadfromSensor', anonymous=True)
        msgsubsriber = rospy.Subscriber("/balboaLL", balboaLLL, self.dealwithValue)
        self.sensor1Value = 0
        self.sensor2Value = 0
        self.sensor3Value = 0
        self.sensor4Value = 0
        self.sensor5Value = 0

    def dealwithValue(self, balboaLLMessage):
        msg = balboaLLL()
        self.sensor1Value = balboaLLMessage.sensor1
        self.sensor2Value = balboaLLMessage.sensor2
        self.sensor3Value = balboaLLMessage.sensor3
        self.sensor4Value = balboaLLMessage.sensor4
        self.sensor5Value = balboaLLMessage.sensor5
        self.sensorValue = [self.sensor1Value,
                            self.sensor2Value,
                            self.sensor3Value,
                            self.sensor4Value,
                            self.sensor5Value]
        # TODO


if __name__ == '__main__':
    node = readFromSensorNode()
