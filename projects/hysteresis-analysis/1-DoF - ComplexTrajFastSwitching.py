# -*- coding: utf-8 -*-
"""
Created on Sat Aug 27 13:15:41 2016

@author: agirard
"""

from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.dynamic  import Prototypes           as Proto
from AlexRobotics.control  import RminComputedTorque   as RminCTC
from AlexRobotics.dynamic  import DynamicSystem        as DS

import numpy as np
import matplotlib
import matplotlib.pyplot as plt


""" Modes """

save_fig      = False
all_fig       = 'output/1link_xu.pdf'
ReComputeTraj = False
name_traj     = 'data/pendulum_traj.npy'


""" Define dynamic system """

R      =  Proto.SingleRevoluteDSDM()
R.ubar =  np.array([0.0,474])

R.M             = 1
R.ext_cst_force = 0

""" Define control model """

R_ctl      =  Proto.SingleRevoluteDSDM()
R_ctl.ubar =  np.array([0.0,474])

R_ctl.M             = 1
R_ctl.ext_cst_force = 0

""" Define control problem """

x_start = np.array([-3.0,0])
x_goal  = np.array([0,0])




""" Trajectory Planning """

RRT = RPRT.RRT( R_ctl , x_start )

T    = 0.01
u_R1 = R_ctl.R[0]
u_R2 = R_ctl.R[1]

RRT.U = np.array([ [ T , u_R1 ] , [ 0 , u_R1 ] , [ -T , u_R1 ] , [ T , u_R2 ] , [ 0 , u_R2 ] , [ -T , u_R2 ] ])


RRT.dt                    = 0.05
RRT.goal_radius           = 0.2
RRT.alpha                 = 0.8
RRT.max_nodes             = 8000
RRT.max_solution_time     = 5

# Make sure no low-gear is used at high-speed by the planner
RRT.test_u_domain = True

# Dynamic plot
RRT.dyna_plot             = False
RRT.dyna_node_no_update   = 1000
RRT.y1axis = 0
RRT.y2axis = 1

#RRT.low_pass_filter.set_freq_to( fc = 5 , dt = RRT.dt )



if ReComputeTraj:
    
    RRT.find_path_to_goal( x_goal )
    RRT.plot_2D_Tree()
    RRT.save_solution( name_traj  )
    print 'Solution Saved'

else:
    
    RRT.load_solution( name_traj  )
    print 'Solution Loaded'
    
    

"""  Assign controller """

CTC_controller     = RminCTC.RminComputedTorqueController( R_ctl )

CTC_controller.load_trajectory( RRT.solution )

CTC_controller.goal         = x_goal
CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.n_gears      = 2
#CTC_controller.traj_ref_pts = 'closest'
#CTC_controller.traj_ref_pts = 'interpol'
CTC_controller.hysteresis   = False
CTC_controller.hys_level    = 0#1.0
CTC_controller.min_delay    = 0.1

R.ctl              = CTC_controller.ctl

""" Simulation """

tf = 5

R.computeSim( x_start , tf , n = int( 10/0.001 ) + 1 , solver = 'euler' )  

R.animateSim()


""" Plot """

R.Sim.fontsize = 7
t_ticks        = [0,2,4,6,8,10]

R.Sim.plot_CL()

R.Sim.plots[0].set_yticks( [-4,0] )
R.Sim.plots[1].set_yticks( [-4,0, 4] )
#R.Sim.plots[2].set_yticks( [-8,0,8] )
R.Sim.plots[3].set_ylim(    0,500 )
R.Sim.plots[3].set_yticks( [23.2,474] )
R.Sim.plots[3].set_xlim(    0,10 )
R.Sim.plots[3].set_xticks( t_ticks )
R.Sim.fig.set_size_inches(4,2.5)
R.Sim.fig.canvas.draw()


if save_fig:
    R.Sim.fig.savefig( all_fig , format='pdf', bbox_inches='tight', pad_inches=0.05)


