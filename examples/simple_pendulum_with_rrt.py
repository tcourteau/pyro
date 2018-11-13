# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.dynamic import pendulum
from AlexRobotics.planning import randomtree

sys  = pendulum.SinglePendulum()

x_start = np.array([0.1,0])
x_goal  = np.array([-3.14,0])

planner = randomtree.RRT( sys , x_start )

planner.u_options = [
            np.array([-5]),
            np.array([-3]),
            np.array([-1]),
            np.array([ 0]),
            np.array([ 1]),
            np.array([ 3]),
            np.array([ 5])
            ]

planner.find_path_to_goal( x_goal )

planner.goal_radius = 0.001

planner.plot_tree()
planner.plot_open_loop_solution()
sys.animate_simulation()