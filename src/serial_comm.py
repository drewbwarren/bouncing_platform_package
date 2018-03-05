#!/usr/bin/env python

import serial
import rospy
from geometry_msgs.msg import Point
from math import floor

class serial_link:


    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0',9600,timeout=10)
        self.sub = rospy.Subscriber('controls',Point,self.control_callback)
        self.timer = rospy.Timer(rospy.Duration(.01),self.timercb)
        self.theta = 0
        self.phi = 0

    def control_callback(self, ctrl):
        self.theta = ctrl.x
        self.phi = ctrl.y
        
    def timercb(self,event):
        self.ser.write('<' +  str(self.truncate(self.theta,5)) + 't')
        self.ser.write('<' +  str(self.truncate(self.phi,5)) + 'p')
        #self.ser.write('<' + str(0.00) + 't')

    def truncate(self, f, n):
        return floor(f * 10 ** n) / 10 ** n

if __name__ == '__main__':
    rospy.init_node('serial_to_arduino',anonymous=True)
    sl = serial_link()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down now")
