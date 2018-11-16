# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.dynamic  import pendulum
from AlexRobotics.planning import randomtree
from AlexRobotics.planning import plan


# Dynamic sysem
sys  = pendulum.SinglePendulum()

# Define planning problem
x_start = np.array([0.1,0])

planner = randomtree.RRT( sys , x_start )

# Solve Planning Problem
#planner.find_path_to_goal( x_goal )

# Print solution
#planner.save_solution('rrt_solution_pendulum.npy')
planner.load_solution('rrt_solution_pendulum.npy')
planner.plot_open_loop_solution()
planner.filter_solution( 1 )


plan =  planner.trajectory