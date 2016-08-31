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

R.x_ub[0] = np.pi
R.x_ub[1] = np.pi
R.x_ub[2] = np.pi
R.x_lb[0] = - np.pi
R.x_lb[1] = - np.pi
R.x_lb[2] = - np.pi

R.ubar = np.array([0,0,0,0])

x_start = np.array([0,0,1.5,0,0,0])
x_goal  = np.array([-3,0,-1.5,0,0,0])

RRT = RPRT.RRT( R , x_start )

T    = 5
u_R1 = 0
u_R2 = 1

RRT.U = np.array([[ 0,T,0,u_R1],[ 0,0,0,u_R1],[ 0,-T,0,u_R1],[ 0,0,T,u_R1],[ 0,0,-T,u_R1],[ 0,T,T,u_R1],[ 0,-T,-T,u_R1],[ 0,-T,T,u_R1],[ 0,T,-T,u_R1],
                  [ T,T,0,u_R1],[ T,0,0,u_R1],[ T,-T,0,u_R1],[ T,0,T,u_R1],[ T,0,-T,u_R1],[ T,T,T,u_R1],[ T,-T,-T,u_R1],[ T,-T,T,u_R1],[ T,T,-T,u_R1],
                  [-T,T,0,u_R1],[-T,0,0,u_R1],[-T,-T,0,u_R1],[-T,0,T,u_R1],[-T,0,-T,u_R1],[-T,T,T,u_R1],[-T,-T,-T,u_R1],[-T,-T,T,u_R1],[-T,T,-T,u_R1],
                  [ 0,T,0,u_R2],[ 0,0,0,u_R2],[ 0,-T,0,u_R2],[ 0,0,T,u_R2],[ 0,0,-T,u_R2],[ 0,T,T,u_R2],[ 0,-T,-T,u_R2],[ 0,-T,T,u_R2],[ 0,T,-T,u_R2],
                  [ T,T,0,u_R2],[ T,0,0,u_R2],[ T,-T,0,u_R2],[ T,0,T,u_R2],[ T,0,-T,u_R2],[ T,T,T,u_R2],[ T,-T,-T,u_R2],[ T,-T,T,u_R2],[ T,T,-T,u_R2],
                  [-T,T,0,u_R2],[-T,0,0,u_R2],[-T,-T,0,u_R2],[-T,0,T,u_R2],[-T,0,-T,u_R2],[-T,T,T,u_R2],[-T,-T,-T,u_R2],[-T,-T,T,u_R2],[-T,T,-T,u_R2]],)


RRT.dt                    = 0.1
RRT.goal_radius           = 0.5
RRT.alpha                 = 0.9
RRT.max_nodes             = 50000
RRT.max_solution_time     = 12

# Dynamic plot
RRT.dyna_plot             = True
RRT.dyna_node_no_update   = 200

RRT.find_path_to_goal( x_goal )

#RRT.animate3D_solution( 0.5 )


# Assign controller
CTC_controller      = RminCTC.RminComputedTorqueController( R )
CTC_controller.load_trajectory( RRT.solution )
CTC_controller.goal = x_goal
R.ctl               = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.traj_ref_pts = 'closest'
CTC_controller.n_gears      = 2

""" Simulation and plotting """

# Sim
tf = RRT.time_to_goal + 5
n  = int( np.round( tf / 0.01 ) ) + 1
R.computeSim( x_start , tf  , n , solver = 'euler' )

# Plot
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
RRT.plot_2D_Tree()
R.animate3DSim()

# Hold figures alive
plt.show()