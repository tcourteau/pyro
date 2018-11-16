# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.dynamic  import pendulum
from AlexRobotics.planning import randomtree

# Dynamic system
sys  = pendulum.SinglePendulum()

# Openloop controller loading RRT solution
ctl = randomtree.OpenLoopController( sys )
ctl.load_solution('rrt_solution_pendulum.npy')

# Closing the loop
cl_sys = ctl + sys

# Simulation
x_start = np.array([0.1,0])
cl_sys.plot_trajectory_CL(  x_start )
cl_sys.animate_simulation()