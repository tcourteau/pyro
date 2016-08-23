# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import RminComputedTorque   as RminCTC

import numpy as np
import matplotlib.pyplot as plt

R  =  HM.HybridThreeLinkManipulator()

R.ubar = np.array([0,0,0,3])

x_start = np.array([0,0,0,0,0,0])
x_goal  = np.array([-3,-1.5,0,0,0,0])

RRT = RPRT.RRT( R , x_start )

T   = 12
u_R = 3

RRT.U = np.array([[0,T,0,u_R],[0,0,0,u_R],[0,-T,0,u_R],[0,0,T,u_R],[0,0,-T,u_R],[0,T,T,u_R],[0,-T,-T,u_R],[0,-T,T,u_R],[0,T,-T,u_R],
                  [T,T,0,u_R],[T,0,0,u_R],[T,-T,0,u_R],[T,0,T,u_R],[T,0,-T,u_R],[T,T,T,u_R],[T,-T,-T,u_R],[T,-T,T,u_R],[T,T,-T,u_R],
                  [-T,T,0,u_R],[-T,0,0,u_R],[-T,-T,0,u_R],[-T,0,T,u_R],[-T,0,-T,u_R],[-T,T,T,u_R],[-T,-T,-T,u_R],[-T,-T,T,u_R],[-T,T,-T,u_R]])


RRT.dt                    = 0.2
RRT.goal_radius           = 0.8
RRT.alpha                 = 0.5
RRT.max_nodes             = 20000
RRT.max_solution_time     = 12


RRT.find_path_to_goal( x_goal )

RRT.animate3D_solution( 0.5 )


# Assign controller
CTC_controller     = RminCTC.RminComputedTorqueController( R )
CTC_controller.load_trajectory( RRT.solution )
CTC_controller.goal = x_goal
R.ctl              = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.traj_ref_pts = 'closest'

""" Simulation and plotting """

# Plot
tf = RRT.time_to_goal + 5
n  = int( np.round( tf / 0.01 ) ) + 1
R.plotAnimation( x_start , tf  , n , solver = 'euler' )
#R.plot3DAnimation( x_start , tf  , n  )
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
RRT.plot_2D_Tree()

# Hold figures alive
plt.show()