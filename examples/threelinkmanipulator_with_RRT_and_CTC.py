# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree    as RPRT
from AlexRobotics.dynamic  import Manipulator   as M
from AlexRobotics.control  import ComputedTorque as CTC

import numpy as np
import matplotlib.pyplot as plt

""" Define system """

R  =  M.ThreeLinkManipulator()

q_start  = np.array([     0              ,   + np.pi * 0.1 ,  + np.pi * 0.1 ])
q_goal   = np.array([     + np.pi * 1    ,   - np.pi * 0.1 ,  - np.pi * 0.1 ])
dq_start = np.array([     0,   0           , 0 ])
dq_goal  = np.array([     0,   0           , 0 ])

x_start = R.q2x( q_start , dq_start )
x_goal  = R.q2x( q_goal  , dq_goal  )

RRT = RPRT.RRT( R , x_start )

T = 8 # torque

RRT.U = np.array([[ 0,T,0],[ 0,0,0],[ 0,-T,0],[ 0,0,T],[ 0,0,-T],[ 0,T,T],[ 0,-T,-T],[ 0,-T,T],[ 0,T,-T],
                  [ T,T,0],[ T,0,0],[ T,-T,0],[ T,0,T],[ T,0,-T],[ T,T,T],[ T,-T,-T],[ T,-T,T],[ T,T,-T],
                  [-T,T,0],[-T,0,0],[-T,-T,0],[-T,0,T],[-T,0,-T],[-T,T,T],[-T,-T,-T],[-T,-T,T],[-T,T,-T]])


RRT.dt                    = 0.2
RRT.goal_radius           = 1.5
RRT.max_nodes             = 25000
RRT.max_solution_time     = 10

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )


# Assign controller
CTC_controller      = CTC.ComputedTorqueController( R )
CTC_controller.load_trajectory( RRT.solution )
CTC_controller.goal = x_goal
R.ctl               = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.traj_ref_pts = 'closest'

""" Simulation and plotting """

# Plot
tf = RRT.time_to_goal + 5
n  = int( np.round( tf / 0.01 ) ) + 1
#R.plotAnimation( x_start , tf  , n   )
R.plot3DAnimation( x_start , tf  , n  )
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
RRT.plot_2D_Tree()

# Hold figures alive
plt.show()