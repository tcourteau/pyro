# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree    as RPRT
from AlexRobotics.dynamic  import Manipulator   as M

import numpy as np
import matplotlib.pyplot as plt

R  =  M.OneLinkManipulator()

tmax = 10
        
R.u_ub = np.array([ tmax])      # Control Upper Bounds
R.u_lb = np.array([-tmax])      # Control Lower Bounds

x_start = np.array([-3.0,0])
x_goal  = np.array([0,0])

RRT = RPRT.RRT( R , x_start )

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )


# Assign controller
#R.ctl = RRT.open_loop_controller
R.ctl = RRT.trajectory_controller

# Plot
tf = RRT.time_to_goal + 5
n = int( tf / RRT.dt )
R.plotAnimation( x_start , tf , n )
R.phase_plane_trajectory([0],x_start,tf,True,False,False,True)
RRT.plot_2D_Tree()
R.Sim.plot_CL()


# Hold figures alive
plt.show()