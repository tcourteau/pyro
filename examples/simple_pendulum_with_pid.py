# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:05:08 2018

@author: Alexandre
"""
###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic import pendulum
from pyro.control import linear
###############################################################################

sys  = pendulum.SinglePendulum()

kp = 4
kd = 1
ki = 0

ctl  = linear.JointPID(sys, kp , ki, kd)

# Set Point
q_target = np.array([3.14])
ctl.rbar = q_target

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x_start  = np.array([0,0])
cl_sys.plot_phase_plane_trajectory( x_start , 10 )
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()