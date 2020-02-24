#!/usr/bin/env python
import struct
import serial
import sys
import rospy
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

class TheNode(object):
  '''Class to communicate with the balboa robot. Carrick Detweiler 2020'''

  #Reads one byte and updates the checksum
  def readOneByteAndChecksum(self):
     c = ord(self.port.read())
     self.checksum = (self.checksum + c) & 0xff
     return c

  #Read four bytes in MSB order and update the checksum
  def readFourByteAndChecksum(self):
    tmp = bytearray(4)
    tmp[0] = ord(self.port.read())
    tmp[1] = ord(self.port.read())
    tmp[2] = ord(self.port.read())
    tmp[3] = ord(self.port.read())
    self.checksum = (self.checksum + tmp[0]) & 0xFF
    self.checksum = (self.checksum + tmp[1]) & 0xFF
    self.checksum = (self.checksum + tmp[2]) & 0xFF
    self.checksum = (self.checksum + tmp[3]) & 0xFF
    val = struct.unpack(">i",tmp)
    return val[0]

  def handleSetMotorSpeed(self,speeds):
    #Verify bounded
    left = int(speeds.left & 0xFF)
    right = int(speeds.right & 0xFF)

    #Start byte (just one)
    self.port.write(chr(0xCD));
    self.port.write(chr(left))
    self.port.write(chr(right))
    #checksum
    self.port.write(chr((0xCD + left + right) & 0xff))

  def __init__(self):

    rospy.init_node('balboa_serial')

    # Make sure to do chmod 777 /dev/ttyACM0 first to make it r/w
    port_file = '/dev/ttyACM0'
    baud = 57600

    self.port = serial.Serial(port=port_file, baudrate=baud)
    self.publisher = rospy.Publisher('balboaLL', balboaLL, queue_size=10)
    rospy.Subscriber("motorSpeeds",balboaMotorSpeeds,self.handleSetMotorSpeed)


  def main_loop(self):
    '''main loop get a packet from the port and parse it and publish it'''
    r = rospy.Rate(200)

    # Publish each byte as it comes in
    while not rospy.is_shutdown():
      # Wait for the start bytes 'CD'
      c = self.port.read()
      while c != 'C':
        sys.stdout.write(c)
        c = self.port.read()
      c = self.port.read()
      if c != 'D':
        sys.stdout.write(c)
        continue

      #Init the checksum
      self.checksum = 0xCD

      msg = balboaLL()
      msg.header.stamp = rospy.Time.now()
      msg.arduinoMillis = self.readFourByteAndChecksum()
      msg.batteryMillivolts = self.readFourByteAndChecksum()
      msg.angleY = self.readFourByteAndChecksum()
      msg.angleX = self.readFourByteAndChecksum()
      msg.angleZ = self.readFourByteAndChecksum()
      msg.driveLeft = self.readFourByteAndChecksum()
      msg.driveRight = self.readFourByteAndChecksum()
      msg.speedLeft = self.readFourByteAndChecksum()
      msg.speedRight = self.readFourByteAndChecksum()
      msg.distanceLeft = self.readFourByteAndChecksum()
      msg.distanceRight = self.readFourByteAndChecksum()

      # print("~~~~~~~")
      # print "should be 0: ", self.readFourByteAndChecksum()
      # print "should be -1: ", self.readFourByteAndChecksum()
      # print "should be 1: ",  self.readFourByteAndChecksum()
      # print "should be int_max: ", self.readFourByteAndChecksum()
      # print "should be int_min: ", self.readFourByteAndChecksum()
      # print "~~~~~~~"

      #verify the checksum
      c = ord(self.port.read())
      if c != self.checksum:
        print "Error, invalid checksum, expected", c, "got", self.checksum
        continue

      #Publish
      self.publisher.publish(msg)

      r.sleep()


if __name__ == '__main__':
  a = TheNode()
  a.main_loop()
