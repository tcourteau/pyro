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

R  =  M.ThreeLinkManipulator()

x_start = np.array([0,   np.pi * 0.5 ,0,0,0,0])
x_goal  = np.array([np.pi, - np.pi * 0.5 ,0,0,0,0])

RRT = RPRT.RRT( R , x_start )

T = 12 # torque

RRT.U = np.array([[0,T,0],[0,0,0],[0,-T,0],[0,0,T],[0,0,-T],[0,T,T],[0,-T,-T],[0,-T,T],[0,T,-T],
                  [T,T,0],[T,0,0],[T,-T,0],[T,0,T],[T,0,-T],[T,T,T],[T,-T,-T],[T,-T,T],[T,T,-T],
                  [-T,T,0],[-T,0,0],[-T,-T,0],[-T,0,T],[-T,0,-T],[-T,T,T],[-T,-T,-T],[-T,-T,T],[-T,T,-T]])


RRT.dt                    = 0.2
RRT.goal_radius           = 2.0
RRT.max_nodes             = 25000
RRT.max_solution_time     = 20

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )


# Assign controller
#R.ctl = RRT.open_loop_controller
R.ctl = RRT.open_loop_controller
RRT.traj_ctl_kp   = 50
RRT.traj_ctl_kd   = 10

""" Simulation and plotting """

# Plot
tf = RRT.time_to_goal + 5
n  = int( np.round( tf / 0.05 ) ) + 1
R.plot3DAnimation( x_start , tf  , n )
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
RRT.plot_2D_Tree()

# Hold figures alive
plt.show()