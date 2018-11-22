# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:05:08 2018

@author: Alexandre
"""
###############################################################################
import numpy as np
###############################################################################
from AlexRobotics.dynamic  import pendulum
from AlexRobotics.control  import nonlinear
from AlexRobotics.planning import plan
###############################################################################

sys  = pendulum.SinglePendulum()



# Controller

traj = plan.load_trajectory('pendulum_rrt.npy')

#ctl  = nonlinear.ComputedTorqueController( sys , traj )
ctl  = nonlinear.SlidingModeController( sys , traj )

ctl.lam  = 5
ctl.gain = 2

# goal
ctl.rbar = np.array([-3.14])

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x_start  = np.array([0.1,0])
cl_sys.plot_trajectory(x_start, 5, 10001, 'euler')
cl_sys.animate_simulation()