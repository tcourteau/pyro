# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.dynamic  import pendulum
from AlexRobotics.planning import plan


# Dynamic system
sys  = pendulum.SinglePendulum()

# Openloop controller loading RRT solution
ctl = plan.load_open_loop_controller('pendulum_rrt.npy')

# Closing the loop
cl_sys = ctl + sys

# Simulation
x_start = np.array([0.1,0])
cl_sys.plot_trajectory(  x_start )
cl_sys.animate_simulation()