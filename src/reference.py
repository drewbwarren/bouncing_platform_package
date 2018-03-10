#!/usr/bin/env python

import numpy as np
import rospy
from stewart_platform.msg import Reference_Pos

class signalGenerator:

    def __init__(self, amplitude=1, frq=1, offset=1):
        self.amp = amplitude
        self.frq = frq
        self.offset = offset

        self.pub = rospy.Publisher('/reference_position', Reference_Pos, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.01),self.timercb)
        self.signal = self.square
        self.base_time = rospy.Time.now()
        self.out = 0


    def timercb(self,event):
        self.square()
        ref = Reference_Pos()
        ref.x = self.out
        ref.y = 0
        self.pub.publish(ref)

    def square(self):
        t = (rospy.Time.now() - self.base_time).to_sec()
        if t % (1.0/self.frq) <= 0.5/self.frq:
            self.out = self.amp + self.offset
        else:
            self.out = -self.amp + self.offset

if __name__ == '__main__':

    rospy.init_node('reference')

    sg = signalGenerator(.03,.05,0)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
