# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

###############################################################################
import numpy as np
###############################################################################
from AlexRobotics.dynamic  import pendulum
from AlexRobotics.control  import nonlinear
from AlexRobotics.planning import randomtree
from AlexRobotics.planning import plan
###############################################################################


sys  = pendulum.DoublePendulum()

###############################################################################

x_start = np.array([-3.14,0,0,0])
x_goal  = np.array([0,0,0,0])

planner = randomtree.RRT( sys , x_start )

t = 12
    
planner.u_options = [
        np.array([-t,-t]),
        np.array([-t,+t]),
        np.array([+t,-t]),
        np.array([+t,+t]),
        np.array([ 0,+t]),
        np.array([ 0,-t]),
        np.array([ 0, 0]),
        np.array([+t, 0]),
        np.array([-t, 0])
        ]

planner.goal_radius          = 1.0
planner.dt                   = 0.1
planner.max_nodes            = 12000
planner.max_solution_time    = 8
planner.max_distance_compute = 500

planner.find_path_to_goal( x_goal )

###############################################################################

# Controller

traj = planner.trajectory

ctl  = nonlinear.TrajectoryFollowingComputedTorqueController( sys , traj )

# goal
ctl.rbar = np.array([0,0])


# New cl-dynamic
cl_sys = ctl + sys

# Simultation
cl_sys.plot_phase_plane_trajectory( x_start  )
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()