# -*- coding: utf-8 -*-
"""
Created on Sat Aug 27 13:15:41 2016

@author: agirard
"""


from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import RminComputedTorque   as RminCTC

import numpy as np
import matplotlib.pyplot as plt

""" Modes """

ReComputeTraj = False
name_traj     = 'output/1link_sol.npy'


""" Define dynamic system """

R      =  HM.HybridOneLinkManipulator()
R.ubar = np.array([0.0,1])


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


""" Compute Open-Loop Solution """

if ReComputeTraj:
    
    RRT.find_path_to_goal( x_goal )
    RRT.save_solution( name_traj  )
    
else:
    
    RRT.load_solution( name_traj  )


"""  Assign controller """

CTC_controller     = RminCTC.RminComputedTorqueController( R )

CTC_controller.load_trajectory( RRT.solution )

R.ctl              = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.traj_ref_pts = 'closest'
CTC_controller.n_gears      = 2
#CTC_controller.traj_ref_pts = 'interpol'

""" Simulation """

tf = RRT.time_to_goal + 5

R.computeSim( x_start , tf )  

""" Plot """

R.animateSim()

R.Sim.fontsize = 12
t_ticks = [0,5,10]

# State fig
R.Sim.plot_CL('x')

R.Sim.plots[0].set_yticks( [-4,-2,0] )
R.Sim.plots[1].set_yticks( [-3,0, 3] )
R.Sim.plots[0].set_xticks( t_ticks )
R.Sim.plots[1].set_xticks( t_ticks )
R.Sim.fig.canvas.draw()
R.Sim.fig.savefig( 'aaa.pdf' , format='pdf', bbox_inches='tight', pad_inches=0.05)


R.Sim.plot_CL('u')

R.Sim.plots[0].set_yticks( [-5,0,5] )
R.Sim.plots[1].set_ylim(    -1,11 )
R.Sim.plots[1].set_yticks( [0,10] )
R.Sim.plots[0].set_xticks( t_ticks )
R.Sim.plots[1].set_xticks( t_ticks )
R.Sim.fig.canvas.draw()
R.Sim.fig.savefig( 'bbb.pdf' , format='pdf', bbox_inches='tight', pad_inches=0.05)