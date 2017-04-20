# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import RminComputedTorque   as RminCTC
from AlexRobotics.dynamic  import DynamicSystem        as DS

import numpy as np
import matplotlib.pyplot as plt



""" Define dynamic system """

R      =  HM.HybridOneLinkManipulator()
R.ubar =  np.array([0.0,1])


""" Define control problem """

x_start = np.array([-3.0,0])
x_goal  = np.array([0,0])


""" Planning Params """

RRT = RPRT.RRT( R , x_start )

T1 = 2.0
T2 = 5.0
R1 = 1
R2 = 10

RRT.U = np.array([ [T1,R1],[0.0,R1],[-T1,R1],[T1,R2],[0.0,R2],[-T1,R2],
                   [T2,R1],[0.0,R1],[-T2,R1],[T2,R2],[0.0,R2],[-T2,R2]  ])

RRT.dt                    = 0.1
RRT.goal_radius           = 0.2
RRT.max_nodes             = 15000
RRT.max_solution_time     = 10

RRT.low_pass_filter.set_freq_to( fc = 1.5 , dt = RRT.dt )


""" OFFline planning """

RRT.find_path_to_goal( x_goal )
RRT.save_solution( 'test'  )
RRT.plot_2D_Tree()
RRT.solution_smoothing()
    
    
"""  Assign controller """

CTC_controller     = RminCTC.RminComputedTorqueController( R )

CTC_controller.load_trajectory( RRT.solution )

R.ctl              = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.n_gears      = 2
#CTC_controller.traj_ref_pts = 'closest'
CTC_controller.traj_ref_pts = 'interpol'
CTC_controller.hysteresis   = False
CTC_controller.hys_level    = 1.0
CTC_controller.min_delay    = 0.5


""" Simulation """

tf = RRT.time_to_goal + 5
n  = int( 10/0.001 ) + 1

R.computeSim( x_start , tf , n , solver = 'euler' ) 


""" FIgures """

#R.plotAnimation( x_start , tf , n , solver = 'euler' )

R.Sim.plot_CL()

R.Sim.phase_plane_trajectory(True,False,False,True)


# Hold figures alive
plt.show()