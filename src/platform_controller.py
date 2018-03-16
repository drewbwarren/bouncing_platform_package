#!/usr/bin/env python

import params as P
from PIDcontroller import PIDControl
from geometry_msgs.msg import Point
import rospy
from stewart_platform.msg import Reference_Pos
import numpy as np

class platformController:

    def __init__(self):
        self.xCtrl = PIDControl(P.kp, P.kd, P.ki, P.theta_eq, P.theta_max, P.beta, P.Ts)
        self.yCtrl = PIDControl(P.kp, P.kd, P.ki, P.phi_eq, P.phi_max, P.beta, P.Ts)
        self.sub = rospy.Subscriber('ball_position', Point, self.position_callback)
        self.ctrl_pub = rospy.Publisher('/controls', Point, queue_size=1)
        self.ref_sub = rospy.Subscriber('reference_position', Reference_Pos, self.ref_callback)
        self.refx = 0
        self.refy = 0
        self.state = [0,0]
        self.fullstate = np.array([[0.0],[0.0],[0.0],[0.0]])
        self.ctrls = [0,0]
        self.xdot_p = 0
        self.xddot_p = 0
        self.ydot_p = 0
        self.yddot_p = 0
        self.pubtimer = rospy.Timer(rospy.Duration(P.Ts), self.timercb)


    def u(self, ref, state):
        # ref is the input
        # state is the current state

        x_r = ref[0]
        x = state[0]
        y_r = ref[1]
        y = state[1]

        theta = self.xCtrl.PID(x_r, x, flag=True)
        phi = self.yCtrl.PID(y_r, y, flag=True)

        return [theta, -phi]

    def position_callback(self, pt):

        self.state = [pt.x,pt.y]
        self.fullstate[0] = pt.x
        self.fullstate[1] = pt.y

    def timercb(self, event):
        
        # Feedforward
        self.feedforward()
        
        ref = [self.refx,self.refy]
        self.ctrls = self.u(ref,self.state)
        
        msg = Point()
        msg.x = self.ctrls[0]
        msg.y = self.ctrls[1]
        msg.z = 0

        self.ctrl_pub.publish(msg)

    def feedforward(self):
        dt = P.Ts
        g = 9.81
        
        def derivatives(state,u):
            return np.matrix([[state.item(2)],[state.item(3)],[g*u[0]],[g*u[1]]])

        k1 = derivatives(self.fullstate,self.ctrls)
        k2 = derivatives(self.fullstate + P.Ts/2*k1,self.ctrls)
        k3 = derivatives(self.fullstate + P.Ts/2*k2,self.ctrls)
        k4 = derivatives(self.fullstate + P.Ts*k3,self.ctrls)
        self.fullstate += (P.Ts/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)
        #self.state = [self.fullstate.item(0), self.fullstate.item(1)]
        self.state[0] = self.fullstate.item(0)
        self.state[1] = self.fullstate.item(1)
        
        #self.state[0] = self.state[0] + self.xdot_p*dt
        #xdot = self.xdot_p + self.xddot_p*dt
        #xddot = g*self.ctrls[0]

        #self.state[1] = self.state[1] + self.ydot_p*dt
        #ydot = self.ydot_p + self.yddot_p*dt
        #yddot = g*self.ctrls[1]

        #self.xdot_p = xdot
        #self.xddot_p = xddot
        #self.ydot_p = ydot
        #self.yddot_p = yddot

    def ref_callback(self,ref):
        self.refx = ref.x
        self.refy = ref.y


if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)

    pc = platformController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting Down")
