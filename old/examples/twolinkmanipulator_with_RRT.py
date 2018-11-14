# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree    as RPRT
from AlexRobotics.dynamic  import Manipulator   as M

import numpy as np
import matplotlib.pyplot as plt

""" Define system """

R  =  M.TwoLinkManipulator()

x_start = np.array([3,0,0,0])
x_goal  = np.array([0,0,0,0])

RRT = RPRT.RRT( R , x_start )

T = 12 # torque

RRT.U = np.array([[T,0],[0,0],[-T,0],[0,T],[0,-T],[T,T],[-T,-T],[-T,T],[T,-T]])
#RRT.U = np.array([[0,T],[0,-T],[0,0]]) # Acrobot problem

RRT.dt                    = 0.1
RRT.goal_radius           = 0.8       # --> really large just for quick example
RRT.max_nodes             = 12000
RRT.max_solution_time     = 8

# Dynamic plot
RRT.dyna_plot             = False
RRT.dyna_node_no_update   = 1000

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )

# Assign controller
#R.ctl = RRT.open_loop_controller
R.ctl = RRT.trajectory_controller
RRT.traj_ctl_kp   = 50
RRT.traj_ctl_kd   = 10

""" Simulation and plotting """

# Plot
tf = RRT.time_to_goal + 5
n  = int( np.round( tf / 0.05 ) ) + 1
R.plotAnimation( x_start , tf  , n )
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
#R.Sim.phase_plane_trajectory(True,False,False,False)
RRT.plot_2D_Tree()

# Hold figures alive
plt.show()