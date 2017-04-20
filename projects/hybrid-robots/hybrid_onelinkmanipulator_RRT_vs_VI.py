# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

save_fig      = True
path          = 'data/'

from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import RminComputedTorque   as RminCTC
from AlexRobotics.dynamic  import DynamicSystem        as DS
from AlexRobotics.control  import DPO                  as DPO

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

T1 = 5.0
T2 = 3.0
R1 = 1
R2 = 10

RRT.U = np.array([ [T1,R1],[0.0,R1],[-T1,R1],[T1,R2],[0.0,R2],[-T1,R2],
                   [T2,R1],[0.0,R1],[-T2,R1],[T2,R2],[0.0,R2],[-T2,R2]  ])

RRT.dt                    = 0.1
RRT.goal_radius           = 0.4
RRT.max_nodes             = 8000
RRT.max_solution_time     = 6

RRT.low_pass_filter.set_freq_to( fc = 1.5 , dt = RRT.dt )


""" OFFline RRT planning """

RRT.find_path_to_goal( x_goal )
#RRT.save_solution( 'test'  )
RRT.solution_smoothing()
    
    
""" Trajectory Following Controller """

CTC_controller     = RminCTC.RminComputedTorqueController( R )

CTC_controller.load_trajectory( RRT.solution )

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.n_gears      = 2
CTC_controller.traj_ref_pts = 'interpol'
CTC_controller.hysteresis   = True
CTC_controller.hys_level    = 1.0
CTC_controller.min_delay    = 0.1


""" Value Iteration """

# Define controller
cost_function      = 'quadratic'
ValueIterationAlgo = DPO.ValueIteration_hybrid_1DOF( R , cost_function )

ValueIterationAlgo.first_step()
ValueIterationAlgo.load_data( path + 'R1H' + cost_function )
#ValueIterationAlgo.compute_steps(50)
ValueIterationAlgo.compute_steps(1)
ValueIterationAlgo.save_data( path + 'R1H' + cost_function ) 

#ValueIterationAlgo.plot_raw_nice( 2 )



""" RRT Simulation """

R.ctl = CTC_controller.ctl
tf    = RRT.time_to_goal + 5
n     = int( 10/0.001 ) + 1

R.computeSim( x_start , tf , n , solver = 'euler' )

x_rrt = R.Sim.x_sol_CL
u_rrt = R.Sim.u_sol_CL

R.Sim.plot_CL()

R.Sim.fontsize = 7
t_ticks = [0,2,4,6,8,10]
R.Sim.plots[0].set_ylim(    -6,1 )
R.Sim.plots[0].set_yticks( [-4,0] )
R.Sim.plots[1].set_ylim(    -5,5 )
R.Sim.plots[1].set_yticks( [-4,0, 4] )
R.Sim.plots[2].set_ylim(    -10,10 )
R.Sim.plots[2].set_yticks( [-8,0,8] )
R.Sim.plots[3].set_ylim(    0,11 )
R.Sim.plots[3].set_yticks( [1,10] )
R.Sim.plots[3].set_xticks( t_ticks )
R.Sim.plots[3].set_xlim(    0,10 )
R.Sim.fig.set_size_inches(2,1.5)
R.Sim.fig.canvas.draw()
if save_fig:
    R.Sim.fig.savefig( 'output/vs_rrt.pdf' , format='pdf', bbox_inches='tight', pad_inches=0.05)

# Compute integral cost
R.Sim.Q = np.diag([0,0])
R.Sim.R = np.diag([1,0])

CTC_controller.reset_hysteresis()

R.Sim.compute()

print 'J_rrt: ', R.Sim.J

""" Value Iteration Simulation """

ValueIterationAlgo.assign_interpol_controller() 

R.computeSim( x_start , tf , n , solver = 'euler' )

x_vi = R.Sim.x_sol_CL
u_vi = R.Sim.u_sol_CL

# Compute integral cost
R.Sim.Q = np.diag([0,0])
R.Sim.R = np.diag([1,0])

R.Sim.compute()

R.Sim.plot_CL()

R.Sim.fontsize = 7
t_ticks = [0,2,4,6,8,10]
R.Sim.plots[0].set_ylim(    -6,1 )
R.Sim.plots[0].set_yticks( [-4,0] )
R.Sim.plots[1].set_ylim(    -5,5 )
R.Sim.plots[1].set_yticks( [-4,0, 4] )
R.Sim.plots[2].set_ylim(    -10,10 )
R.Sim.plots[2].set_yticks( [-8,0,8] )
R.Sim.plots[3].set_ylim(    0,11 )
R.Sim.plots[3].set_yticks( [1,10] )
R.Sim.plots[3].set_xticks( t_ticks )
R.Sim.plots[3].set_xlim(    0,10 )
R.Sim.fig.set_size_inches(2,1.5)
R.Sim.fig.canvas.draw()
if save_fig:
    R.Sim.fig.savefig( 'output/vs_vi.pdf' , format='pdf', bbox_inches='tight', pad_inches=0.05)

print 'J_vi: ', R.Sim.J



""" FIgures """

#R.plotAnimation( x_start , tf , n , solver = 'euler' )

#R.Sim.plot_CL()

#R.Sim.phase_plane_trajectory(True,False,False,True)


# Hold figures alive
plt.show()