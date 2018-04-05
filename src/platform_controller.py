#!/usr/bin/env python

import params as P
from PIDcontroller import PIDControl
from geometry_msgs.msg import Point
import rospy
from stewart_platform.msg import Reference_Pos
import numpy as np

class platformController:

    def __init__(self):
        rospy.sleep(10)
        init_cond = rospy.wait_for_message('ball_position', Point)
        self.sub = rospy.Subscriber('ball_position', Point, self.position_callback)
        self.ctrl_pub = rospy.Publisher('/controls', Point, queue_size=1)
        self.ref_sub = rospy.Subscriber('reference_position', Reference_Pos, self.ref_callback)
        self.refx = 0.0
        self.refy = 0.0
        self.x_p = 0.0
        self.y_p = 0.0
        self.xdot = 0.0
        self.ydot = 0.0
        self.K = P.K
        self.limit = P.theta_max
        self.time_prev = rospy.Time.now()
        self.Ts = P.Ts
        self.beta =P.beta
        self.theta_prev = 0.0
        self.phi_prev = 0.0
        self.xstate = np.array([[init_cond.x],[0.0]])
        self.ystate = np.array([[init_cond.y],[0.0]])



    def u(self, ref, state):
        # ref is the input
        # state is the current state
        
        t = rospy.Time.now()
        self.Ts = (t - self.time_prev).to_sec()
        self.beta = (2*0.1 - self.Ts)/(2*0.1 + self.Ts)
        self.time_prev = t

        x_r = ref[0]
        x = state[0]
        y_r = ref[1]
        y = state[1]

        self.differentiate(x,y)

        #rospy.loginfo(self.xstate)

        xstatedot = np.dot(P.A,self.xstate) + np.dot(P.B,np.array([[self.theta_prev],[x]]))
        ystatedot = np.dot(P.A,self.ystate) + np.dot(P.B,np.array([[self.phi_prev],[y]]))
        
        #rospy.loginfo(xstatedot)
        
        self.xstate += np.dot(xstatedot,self.Ts)
        self.ystate += np.dot(ystatedot,self.Ts)

        theta_unsat = -np.dot(self.K,self.xstate - np.array([[x_r],[0.0]]) )
        phi_unsat = -np.dot(self.K,self.ystate - np.array([[y_r],[0.0]]) )
        theta_unsat = theta_unsat[0][0]
        phi_unsat = phi_unsat[0][0]
        
        #rospy.loginfo(xstatedot)
        #rospy.loginfo(self.xstate)
        #rospy.loginfo(theta_unsat) 
        

        theta = self.saturate(theta_unsat)
        phi = self.saturate(phi_unsat)
        
        self.theta_prev = theta
        self.phi_prev = phi

        return [theta, phi]

    def differentiate(self,x,y):
        self.xdot = self.beta*self.xdot + (1-self.beta)*((x - self.x_p)/self.Ts)
        self.ydot = self.beta*self.ydot + (1-self.beta)*((y - self.y_p)/self.Ts)
        
        self.x_p = x
        self.y_p = y

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

    def position_callback(self, pt):

        x = pt.x
        y = pt.y
        state = [x,y]

        ######### Change this once a reference is defined
        ref = [self.refx,self.refy]

        ctrls = self.u(ref,state)
        
        msg = Point()
        msg.x = ctrls[0]
        msg.y = ctrls[1]
        msg.z = 0

        self.ctrl_pub.publish(msg)

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
