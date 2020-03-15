#!/usr/bin/env python
import rospy
# import pandas as pd
from balboa_core.msg import balboaLL
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class readFromSensorNode(object):

    def __init__(self):
        rospy.init_node('ReadfromSensor', anonymous=True)
        self.sensorSub = rospy.Subscriber("/balboaLL", balboaLL, self.handleSensorValues, queue_size = 1)
        self.turnSub = rospy.Subscriber("turnSignal", Float64, self.setTurning, queue_size = 1)
        self.movementSub = rospy.Subscriber('movementCommand', Float64, self.printMap, queue_size = 1)
        self.isTurning = False
        self.isForwards = True
        self.characterMap = []
        self.currentRun = []
        self.table = pd.DataFrame()
        self.lengthfw = 0
        self.lengthbw = 0
        rospy.spin()

    def setTurning(self, value):
        self.isTurning = bool(value.data)


    def handleSensorValues2(self, balboaLLMessage):

        # not turnning, Deal with the sensorValues data by scannining, store them into an array
        if(not self.isTurning):
            sensorValues = self.sensorToString(balboaLLMessage.sensor1) + self.sensorToString(balboaLLMessage.sensor2) + self.sensorToString(balboaLLMessage.sensor3) + self.sensorToString(balboaLLMessage.sensor4) + self.sensorToString(balboaLLMessage.sensor5)
            if(self.isForwards):    # Go forward direction
                self.currentRun.append(sensorValues)
                self.lengthfw = self.lengthfw + 1
            else:                    # Go backward direction
                self.currentRun.insert(0, sensorValues)
                self.lengthbw = self.lengthbw + 1

        # Start turnning
        else:
            self.isForwards = not self.isForwards # Go opposite way to scanning

            if(self.table.empty == False): # If table is not empty
                if(self.lenghfw < self.lengthbw): # If it is smaller than last column
                    self.currentRun.pop(self.lenghbw - self.lenghfw)
                    self.table = self.table.concat(self.currentRun, ignore_index=True)

                elif(self.lenghbw < self.lenthfw): # If it is greater than last column
                    self.currentRun.pop(self.lenghfw - self.lenghbw)
                    self.table = self.table.concat(self.currentRun, ignore_index=True)

                else: # They are equal size
                    self.table = self.table.concat(self.currentRun, ignore_index=True)

                self.lengthfw = 0
                self.lenghbw = 0

            else: # It is empty and just add up the first column into the table
                self.table = self.table.concat(self.currentRun, ignore_index=True)
                print(self.table.head()) ## Debug
                self.currentRun = []
                self.lengthfw = 0
                self.lenghbw = 0


    def handleSensorValues(self, balboaLLMessage):

        # not turnning
        if(not self.isTurning):
            sensorValues = self.sensorToString(balboaLLMessage.sensor1) + self.sensorToString(balboaLLMessage.sensor2) + self.sensorToString(balboaLLMessage.sensor3) + self.sensorToString(balboaLLMessage.sensor4) + self.sensorToString(balboaLLMessage.sensor5)
            if(self.isForwards):
                self.currentRun.append(sensorValues)
            else:
                self.currentRun.insert(0, sensorValues)
        # turnning
        else:
            self.isForwards = not self.isForwards
            if(self.currentRun != []):
                self.characterMap.append(self.currentRun)
            self.currentRun = []

    def sensorToString(self, sensorValue):

        if(sensorValue < 75):
            return " "
        elif(sensorValue < 175):
            return "."
        else:
            return "x"

    def printMap(self, value):
        print(value.data)
        if(value.data == 0.0):
            # print(self.table)
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
                    string5 = string5 + str[4]
                print(string1)
                print(string2)
                print(string3)
                print(string4)
                print(string5)
        if(value.data == 1.0):
            self.currentRun = []


if __name__ == '__main__':
    node = readFromSensorNode()
