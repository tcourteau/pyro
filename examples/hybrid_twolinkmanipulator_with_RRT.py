# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM

import numpy as np
import matplotlib.pyplot as plt

R  =  HM.HybridTwoLinkManipulator()

R.ubar = np.array([0,0,3])

x_start = np.array([3,0,0,0])
x_goal  = np.array([0,0,0,0])

RRT = RPRT.RRT( R , x_start )

T = 5

RRT.U = np.array([[T,0,0],[0,0,0],[-T,0,0],[0,T,0],[0,-T,0],[T,T,0],[-T,-T,0],[-T,T,0],[T,-T,0],
                  [T,0,1],[0,0,1],[-T,0,1],[0,T,1],[0,-T,1],[T,T,1],[-T,-T,1],[-T,T,1],[T,-T,1],
                  [T,0,2],[0,0,2],[-T,0,2],[0,T,2],[0,-T,2],[T,T,2],[-T,-T,2],[-T,T,2],[T,-T,2],
                  [T,0,3],[0,0,3],[-T,0,3],[0,T,3],[0,-T,3],[T,T,3],[-T,-T,3],[-T,T,3],[T,-T,3]])

#RRT.U = np.array([[T,0,3],[0,0,3],[-T,0,3],[0,T,3],[0,-T,3],[T,T,3],[-T,-T,3],[-T,T,3],[T,-T,3]])


RRT.dt                    = 0.2
RRT.goal_radius           = 0.8
RRT.max_nodes             = 10000
RRT.max_solution_time     = 12

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )

# Assign controller
#R.ctl             = RRT.open_loop_controller
R.ctl             = RRT.trajectory_controller # PD Joint controller
RRT.traj_ctl_kp   = 20
RRT.traj_ctl_kd   = 10

# Plot
tf = RRT.time_to_goal + 5
n  = int( np.round( tf / 0.05 ) ) + 1
R.plotAnimation( x_start , tf , n , solver = 'euler' )
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
#R.phase_plane_trajectory([0,0,3],x_start,tf,True,False,False,True)
RRT.plot_2D_Tree()

# Hold figures alive
plt.show()