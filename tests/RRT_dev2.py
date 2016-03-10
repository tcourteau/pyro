# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree    as RPRT
from AlexRobotics.dynamic  import DynamicSystem as RDDS
from AlexRobotics.dynamic  import Manipulator   as M

import numpy as np

R  =  M.TwoLinkManipulator()

x_start = np.array([-3,0,-3,0])
x_goal  = np.array([0,0,0,0])

RRT = RPRT.RRT( R , x_start )

RRT.U = np.array([[10,0],[0,0],[-10,0],[0,10],[0,-10],[10,10],[-10,-10]])

RRT.eps = 1

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )

# Assign controller
#R.ctl = RRT.open_loop_controller
R.ctl = RRT.trajectory_controller

# Plot
tf = RRT.time_to_goal + 5
n = int( tf / RRT.dt )
R.plotAnimation( x_start , tf , n )
R.phase_plane_trajectory([0,0],x_start,tf,True,False,False,True)
RRT.plot_2D_Tree()
R.Sim.plot_CL()


# Hold figures alive
plt.show()