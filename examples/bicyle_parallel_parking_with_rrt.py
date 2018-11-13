# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:20:11 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.dynamic import vehicle
from AlexRobotics.planning import randomtree

sys  = vehicle.KinematicBicyleModel()

x_start = np.array([0,0,0])
x_goal  = np.array([0,1,0])

planner = randomtree.RRT( sys , x_start )

speed    = 1
steering = 0.3

planner.u_options = [
        np.array([ speed,-steering]),
        np.array([ speed,+steering]),
        np.array([ speed,0]),
        np.array([-speed,+steering]),
        np.array([-speed,0]),
        np.array([-speed,-steering])
        ]

planner.find_path_to_goal( x_goal )

planner.plot_tree()
planner.plot_tree_3d()
planner.plot_open_loop_solution()

sys.dynamic_domain = False
sys.animate_simulation()