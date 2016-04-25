# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.control  import RminComputedTorque   as RminCTC

import numpy as np
import matplotlib.pyplot as plt

R  =  HM.HybridTwoLinkManipulator()

R.ubar = np.array([0,0,3])

x_start = np.array([3,0,0,0])
x_goal  = np.array([0,0,0,0])

RRT = RPRT.RRT( R , x_start )

T = 10

RRT.U = np.array([[T,0,0],[0,0,0],[-T,0,0],[0,T,0],[0,-T,0],[T,T,0],[-T,-T,0],[-T,T,0],[T,-T,0],
                  [T,0,1],[0,0,1],[-T,0,1],[0,T,1],[0,-T,1],[T,T,1],[-T,-T,1],[-T,T,1],[T,-T,1],
                  [T,0,2],[0,0,2],[-T,0,2],[0,T,2],[0,-T,2],[T,T,2],[-T,-T,2],[-T,T,2],[T,-T,2],
                  [T,0,3],[0,0,3],[-T,0,3],[0,T,3],[0,-T,3],[T,T,3],[-T,-T,3],[-T,T,3],[T,-T,3]])

#RRT.U = np.array([[T,0,1],[0,0,1],[-T,0,1],[0,T,1],[0,-T,1],[T,T,1],[-T,-T,1],[-T,T,1],[T,-T,1]])
#RRT.U = np.array([[T,0,2],[0,0,2],[-T,0,2],[0,T,2],[0,-T,2],[T,T,2],[-T,-T,2],[-T,T,2],[T,-T,2]])
#RRT.U = np.array([[T,0,3],[0,0,3],[-T,0,3],[0,T,3],[0,-T,3],[T,T,3],[-T,-T,3],[-T,T,3],[T,-T,3]])
#RRT.U = np.array([[T,0,0],[0,0,0],[-T,0,0],[0,T,0],[0,-T,0],[T,T,0],[-T,-T,0],[-T,T,0],[T,-T,0]])
#RRT.U = np.array([[0,T,0],[0,0,0],[0,-T,0]])


RRT.dt                    = 0.2
RRT.goal_radius           = 0.3
RRT.max_nodes             = 12000
RRT.max_solution_time     = 5

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )

# Assign controller

# Assign controller
CTC_controller     = RminCTC.RminComputedTorqueController( R )
CTC_controller.load_trajectory( RRT.solution )
R.ctl              = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.traj_ref_pts = 'closest'
#CTC_controller.traj_ref_pts = 'interpol'

##R.ctl             = RRT.open_loop_controller
#R.ctl             = RRT.trajectory_controller # PD Joint controller
#RRT.traj_ctl_kp   = 20
#RRT.traj_ctl_kd   = 10

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