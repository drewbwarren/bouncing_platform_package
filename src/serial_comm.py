#!/usr/bin/env python

import serial
import rospy
from geometry_msgs.msg import Point
from math import floor

class serial_link:


    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0',9600,timeout=10)
        self.sub = rospy.Subscriber('controls',Point,self.control_callback)


    def control_callback(self, ctrl):
        theta = ctrl.x
        phi = ctrl.y
        self.ser.write(str(self.truncate(theta,7)) + '\n')
        self.ser.write(str(self.truncate(phi,7)) + '\n')
        #rospy.loginfo(str(theta))

    def truncate(self, f, n):
        return floor(f * 10 ** n) / 10 ** n

if __name__ == '__main__':
    rospy.init_node('serial_to_arduino',anonymous=True)
    sl = serial_link()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down now")
