# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 13:02:28 2018
@author: nvidia
"""

import numpy as np

from pyro.dynamic import vehicle
from pyro.planning import randomtree

sys  = vehicle.KinematicBicyleModelwithObstacles()

# Set domain
sys.x_ub = np.array([+10,+10,+6.28])
sys.x_lb = np.array([-10,-10,-6.284])

sys.obstacles = [[ (-2,-1),(2,1)]]
                
        
x_start = np.array([-10,0,0])
x_goal  = np.array([7,0,0])

planner = randomtree.RRT( sys , x_start )

speed    = 1
steering = 0.2

planner.u_options = [
        np.array([ +steering, speed]),
        np.array([ -steering, speed]),
        np.array([ 0, speed]),
        np.array([ 0,-speed])
        ]

planner.dt          = 0.1
planner.steps       = 5
planner.alpha       = 0.9
planner.beta        = 0.1
planner.goal_radius = 2.0

planner.find_path_to_goal( x_goal )

planner.plot_tree()
planner.plot_open_loop_solution()

sys.dynamic_domain = False
sys.animate_simulation()