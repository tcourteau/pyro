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

sys  = pendulum.DoublePendulum()



# Controller

traj = plan.load_trajectory('double_pendulum_rrt.npy')

ctl  = nonlinear.SlidingModeController( sys , traj )

# goal
ctl.rbar = np.array([0,0])

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x_start  = np.array([-3.14,0,0,0])
cl_sys.plot_trajectory( x_start , 10 , 10001, 'euler')
cl_sys.sim.phase_plane_trajectory(0,2)
cl_sys.animate_simulation()