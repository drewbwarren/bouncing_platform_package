#!/usr/bin/env python

import numpy as np
import rospy
from stewart_platform.msg import Reference_Pos
from sensor_msgs.msg import Joy

class signalGenerator:

    def __init__(self, amplitude=1, frq=1, offset=1):
        self.amp = amplitude
        self.frq = frq
        self.offset = offset

        self.pub = rospy.Publisher('/reference_position', Reference_Pos, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.01),self.timercb)
        self.base_time = rospy.Time.now()
        
        self.ref = Reference_Pos()
        self.ref.x = 0
        self.ref.y = 0

        self.signal = [self.static, self.square, self.circle]
        self.ind = 1
        
        self.joy = Joy()
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.limit = .1
        self.joy_scaler = .001

    def timercb(self,event):
        self.signal[self.ind]()
        self.limit_check()
        self.pub.publish(self.ref)

    def square(self):
        t = (rospy.Time.now() - self.base_time).to_sec()
        if t % (1.0/self.frq) <= 0.5/self.frq:
            self.ref.x = self.amp + self.offset
        else:
            self.ref.x = -self.amp + self.offset
        self.ref.y = 0

    def circle(self):
        t = (rospy.Time.now() - self.base_time).to_sec()
        self.ref.x = self.amp*np.cos(2*np.pi*t*self.frq)
        self.ref.y = self.amp*np.sin(2*np.pi*t*self.frq)

    def static(self):
        
        if self.joy.buttons[0]:
            self.ref.x = 0
            self.ref.y = 0
        else:
            if np.abs(self.joy.axes[0]) > .1:
                self.ref.x += -self.joy.axes[0]*self.joy_scaler
            if np.abs(self.joy.axes[1]) > .1:
                self.ref.y += self.joy.axes[1]*self.joy_scaler
        

    def joy_cb(self,joy):

        if joy.buttons[3]:
            self.ind += 1
            rospy.loginfo('Control mode changed')
            if self.ind > len(self.signal) - 1:
                self.ind = 0
            if self.ind == 0:
                self.ref.x = 0
                self.ref.y = 0

        self.joy = joy
        
        

    def limit_check(self):
        if np.abs(self.ref.x) > self.limit:
            self.ref.x = self.limit*np.sign(self.ref.x)
        if np.abs(self.ref.y) > self.limit:
            self.ref.y = self.limit*np.sign(self.ref.y)


if __name__ == '__main__':

    rospy.init_node('reference')

    sg = signalGenerator(.04,.15,0)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
