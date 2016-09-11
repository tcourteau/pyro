# -*- coding: utf-8 -*-
"""
Created on Sat Aug 27 13:15:41 2016

@author: agirard
"""


from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import RminComputedTorque   as RminCTC
from AlexRobotics.dynamic  import DynamicSystem        as DS

import numpy as np
import matplotlib
import matplotlib.pyplot as plt


""" Modes """

ReComputeTraj = False
save_fig      = True
name_traj     = 'data/1link_sol.npy'
all_fig       = 'output/1link_xu.pdf'


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


""" Compute Open-Loop Solution """

if ReComputeTraj:
    
    RRT.find_path_to_goal( x_goal )
    RRT.save_solution( name_traj  )
    RRT.plot_2D_Tree()
    
else:
    
    RRT.load_solution( name_traj  )
    
#RRT.plot_open_loop_solution()
#RRT.plot_open_loop_solution_acc()

RRT.solution_smoothing()

#RRT.plot_open_loop_solution()
#RRT.plot_open_loop_solution_acc()


"""  Assign controller """

CTC_controller     = RminCTC.RminComputedTorqueController( R )

CTC_controller.load_trajectory( RRT.solution )

R.ctl              = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.n_gears      = 2
#CTC_controller.traj_ref_pts = 'closest'
CTC_controller.traj_ref_pts = 'interpol'
CTC_controller.hysteresis   = True
CTC_controller.hys_level    = 1.0
CTC_controller.min_delay    = 0.1

""" Simulation """

tf = RRT.time_to_goal + 5

R.computeSim( x_start , tf , n = int( 10/0.001 ) + 1 , solver = 'euler' )  

""" Plot """

R.Sim.fontsize = 7
t_ticks = [0,5,10]



R.Sim.plot_CL()

R.Sim.plots[0].set_yticks( [-4,0] )
R.Sim.plots[1].set_yticks( [-4,0, 4] )
R.Sim.plots[2].set_yticks( [-8,0,8] )
R.Sim.plots[3].set_ylim(    0,11 )
R.Sim.plots[3].set_yticks( [1,10] )
R.Sim.plots[3].set_xlim(    0,10 )
R.Sim.plots[3].set_xticks( t_ticks )
R.Sim.fig.canvas.draw()
if save_fig:
    R.Sim.fig.savefig( all_fig , format='pdf', bbox_inches='tight', pad_inches=0.05)

# phase plane 1
PP1 =  DS.PhasePlot( R )

PP1.y1max = 2
PP1.y1min = -5
PP1.y2max = 4
PP1.y2min = -4

name = 'output/simpp1' + '.pdf'

PP1.CL         = False
PP1.color_CL   = 'b'
PP1.linewidth  = 0.04
PP1.headlength = 3.5
PP1.fontsize   = 7
PP1.dpi        = 600
PP1.figsize    = (3,2)
PP1.y1n        = 11
PP1.y2n        = 11

PP1.u          = np.array([0,1])
PP1.compute()
PP1.plot( R.Sim )

if save_fig:
    PP1.phasefig.savefig( name , format = 'pdf' , bbox_inches='tight', pad_inches=0.05 )

# phase plane 1
PP2 =  DS.PhasePlot( R )

PP2.y1max = 2
PP2.y1min = -5
PP2.y2max = 4
PP2.y2min = -4

name = 'output/simpp2' + '.pdf'

PP2.CL         = False
PP2.color_CL   = 'b'
PP2.linewidth  = 0.04
PP2.headlength = 3.5
PP2.fontsize   = 7
PP2.dpi        = 600
PP2.figsize    = (3,2)
PP2.y1n        = 11
PP2.y2n        = 11

PP2.u          = np.array([0,10])
PP2.compute()
PP2.plot( R.Sim )

if save_fig:
    PP2.phasefig.savefig( name , format = 'pdf' , bbox_inches='tight', pad_inches=0.05 )

name_video = 'output/sim1_anim'

R.animateSim( 1.0 , save_fig ,  name_video )

plt.show()

n = R.Sim.n

# Compute integral cost
R.Sim.Q = np.diag([0,0])
R.Sim.R = np.diag([1,0])

R.Sim.compute()

print 'max torque gearshift:' , R.Sim.u_sol_CL[:,0].max()
print 'min torque gearshift:' , R.Sim.u_sol_CL[:,0].min()
print '      cost gearshift:' , R.Sim.J

R.Sim.plot_CL('u')

############

CTC_controller.last_gear_i = 0
CTC_controller.n_gears = 1
R.R = [ np.diag([1]) ,   np.diag([1]) ]

R.Sim.compute()

print 'max torque 1:1 :' , R.Sim.u_sol_CL[:,0].max()
print 'min torque 1:1 :' , R.Sim.u_sol_CL[:,0].min()
print '      cost 1:1 :' , R.Sim.J

R.Sim.plot_CL('u')

############

CTC_controller.n_gears = 1
R.R = [ np.diag([10]) , np.diag([10]) ]

R.Sim.compute()

print 'max torque 1:10 :' , R.Sim.u_sol_CL[:,0].max()
print 'min torque 1:10 :' , R.Sim.u_sol_CL[:,0].min()
print '      cost 1:10 :' , R.Sim.J

R.Sim.plot_CL('u')