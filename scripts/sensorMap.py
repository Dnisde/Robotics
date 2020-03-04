#!/usr/bin/env python
import rospy
import numpy as np
from lab2.msg import balboaLL
# from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

class readFromSensorNode(object):

    def __init__(self):
        rospy.init_node('ReadfromSensor', anonymous=True)
        self.sensorSub = rospy.Subscriber("/balboaLL", balboaLL, self.dealwithValue, queue_size = 1)
        self.turnSub = rospy.Subscriber("turnSignal", Bool, self.setTurning, queue_size = 1)
        self.movementSub = rospy.Subscriber('movementCommand', String, self.printMap, queue_size = 1)    
        self.isTurning = False
        self.isForwards = True
        self.characterMap = []
        self.currentRun = []

    def setTurning(self, value):
        self.isTurning = value

    def handleSensorValues(self, balboaLLMessage):
        if(!self.isTurning):
            sensorValues = chr(balboaLLMessage.sensor1) + chr(balboaLLMessage.sensor2)+ chr(balboaLLMessage.sensor3)+ chr(balboaLLMessage.sensor4)+ chr(balboaLLMessage.sensor5)
            if(self.isForwards):
                self.currentRun.append(sensorValues)
            else:
                self.currentRun.insert(0, sensorValues)
        else:
            self.isForwards = not self.isForwards
            if(self.currentRun != []):
                self.characterMap.append(self.currentRun)
            self.currentRun = []
    
    def printMap(self, value):
        if(value == "stop"):
            for list in self.characterMap:
                string1 = ''
                string2 = ''
                string3 = ''
                string4 = ''
                string5 = ''
                for str in list:
                    string1 = string1 + str[0]
                    string2 = string2 + str[1]
                    string3 = string3 + str[2]
                    string4 = string4 + str[3]
                    string5 = string5 + str[5]
                print(string1)
                print(string2)
                print(string3)
                print(string4)
                print(string5)


if __name__ == '__main__':
    node = readFromSensorNode()
