#!/usr/bin/env python

import serial
import rospy

class resetter:

    def __init__(self):
        ser = serial.Serial('/dev/ttyACM0',9600,timeout=10)
        default_theta = 0.0
        default_phi = 0.0
        theta = rospy.get_param("~theta",default_theta)
        phi = rospy.get_param("~phi",default_phi)

        for i in range(1000):
            ser.write('<' + str(theta) + 't')
            ser.write('<' + str(phi) + 'p')


if __name__ == '__main__':
    rospy.init_node('reset_platform')
    r = resetter()
