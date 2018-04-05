
import numpy as np
import control as cnt

# System Parameters
g = 9.81

# Animation Parameters
radius = .015

# Initial Conditions
x0 = 0.0
y0 = 0.0
xdot0 = 0.0
ydot0 = 0.0

# Simulation Parameters
t_start = 0.0
t_end = 20.0
Ts = 0.01
t_plot = 0.1

# Limits
theta_max = 10*3.1415/180
phi_max = theta_max


# Equilibrium
theta_eq = 0.0427194530199  
phi_eq = 0.0324069722134  
phi_eq = -0.03
#theta_eq = 1.2*3.1415/180.0
#phi_eq = -0.8*3.1415/180.0

# Controller Gains
tr = .2
zeta = .707
wn = 2.2/tr

#kp = 0.6 #.8 #(wn**2)/g
#kd = 1 #.2 #(2*zeta*wn)/g

kp = 0.5
kd = 1. 
ki = 0.01

#kp = 0.2
#kd = 0.8
#ki = 0

# Dirty Derivative Parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2*sigma - Ts)/(2*sigma + Ts)  # dirty derivative gain




### State Space stuff
A = np.array([[0.0,1.0],[0.0,0.0]])
B = np.array([[0.0],[9.81]])

Q = np.array([[12.0,0.0],[0.0,1.0]])
R = np.array([[10.0]])

Q = np.array([[2000.0,0.0],[0.0,1.0]])
R = np.array([[5.0]])

K,_,_ = cnt.lqr(A,B,Q,R)

C = np.array([[1.0,0.0]])
Vd = np.eye(2)*1.0
Vn = np.array([[0.01]])
Kf,_,_ = cnt.lqr(A.T,C.T,Vd,Vn)
Kf = Kf.T

A = A - np.dot(Kf,C)
B = np.concatenate((B,Kf),axis=1)
