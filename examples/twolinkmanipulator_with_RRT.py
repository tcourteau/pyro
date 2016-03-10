# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree    as RPRT
from AlexRobotics.dynamic  import Manipulator   as M

import numpy as np
import matplotlib.pyplot as plt

R  =  M.TwoLinkManipulator()

x_start = np.array([3,0,0,0])
x_goal  = np.array([0,0,0,0])

RRT = RPRT.RRT( R , x_start )

RRT.U = np.array([[5,0],[0,0],[-5,0],[0,5],[0,-5],[5,5],[-5,-5],[-5,5],[5,-5]])

RRT.dt           = 0.1
RRT.goal_radius  = 1
RRT.max_nodes    = 50000

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )

# Assign controller
#R.ctl = RRT.open_loop_controller
R.ctl = RRT.trajectory_controller

# Plot
RRT.traj_ctl_kp   = 50
RRT.traj_ctl_kd   = 10
tf                = RRT.time_to_goal + 5
R.plotAnimation( x_start , tf )
R.phase_plane_trajectory([0,0],x_start,tf,True,False,False,True)
RRT.plot_2D_Tree()
R.Sim.plot_CL()

# Hold figures alive
plt.show()