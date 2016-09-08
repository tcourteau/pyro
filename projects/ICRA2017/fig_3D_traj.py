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


test_name     = 'no4'
ReComputeTraj = False
save_fig      = True
name_traj     = 'data/3D_sol_'+ test_name +'.npy'


####################################
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


RRT.dt                    = 0.15
RRT.goal_radius           = 0.6
RRT.alpha                 = 0.8
RRT.max_nodes             = 25000
RRT.max_solution_time     = 3

# Dynamic plot
RRT.dyna_plot             = True
RRT.dyna_node_no_update   = 1000

RRT.low_pass_filter.set_freq_to( fc = 2.0 , dt = RRT.dt )


if ReComputeTraj:
    
    RRT.find_path_to_goal( x_goal )
    RRT.save_solution( name_traj  )
    #RRT.plot_2D_Tree()
    
else:
    
    RRT.load_solution( name_traj  )
    

#RRT.plot_open_loop_solution()
#RRT.plot_open_loop_solution_acc()

RRT.solution_smoothing()

#RRT.plot_open_loop_solution()
#RRT.plot_open_loop_solution_acc()


# Assign controller
CTC_controller      = RminCTC.RminComputedTorqueController( R )
CTC_controller.load_trajectory( RRT.solution )
CTC_controller.goal = x_goal
R.ctl               = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.traj_ref_pts = 'interpol'
CTC_controller.n_gears      = 8
CTC_controller.hysteresis   = True
CTC_controller.hys_level    = 3

""" Simulation and plotting """

# Sim
tf = RRT.time_to_goal + 5
n  = int( np.round( tf / 0.01 ) ) + 1
R.computeSim( x_start , tf  , n , solver = 'euler' )

# Plot

R.Sim.fontsize = 7


R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')

R.Sim.plots[0].set_ylim( -12,12 )
R.Sim.plots[1].set_ylim( -12,12 )
R.Sim.plots[2].set_ylim( -12,12 )
R.Sim.plots[0].set_yticks( [-10,0,10] )
R.Sim.plots[1].set_yticks( [-10,0,10] )
R.Sim.plots[2].set_yticks( [-10,0,10]  )
R.Sim.plots[3].set_yticks( [0,8] )
R.Sim.plots[3].set_ylim( -0.5,8.5 )
R.Sim.plots[3].set_xlim( -0,6 )

R.Sim.fig.canvas.draw()

if save_fig:
    R.Sim.fig.savefig( 'output/' + 'u_'+test_name+ '.pdf' , format='pdf', bbox_inches='tight', pad_inches=0.05)

if save_fig:
    
    R.animate3DSim( 1.0 , True ,  'output/' + test_name )
    
else:
    
    R.animate3DSim()


R.lw = 2.5
R.show_traj_3D([180,275,350,400,500])
R.ax_show_3D.elev = 35
R.ax_show_3D.azim = 35
R.ax_show_3D.set_xlabel('')
R.ax_show_3D.set_ylabel('')
R.ax_show_3D.set_zlabel('')
R.ax_show_3D.set_xlim3d([ - R.lw / 2. , R.lw / 2. ])
R.ax_show_3D.set_ylim3d([ - R.lw / 2. + 0.5 , R.lw / 2. + 0.5 ])
R.ax_show_3D.set_zlim3d([ - R.lw / 2. + 1.0 , R.lw / 2. + 1.0 ])
R.ax_show_3D.set_xticklabels([])
R.ax_show_3D.set_yticklabels([])
R.ax_show_3D.set_zticklabels([])
R.fig_show_3D.canvas.draw()

if save_fig:

    R.fig_show_3D.savefig( 'output/' + '3d_traj_'+test_name+ '.pdf' , format = 'pdf' , bbox_inches='tight', pad_inches=0.05 )

# Hold figures alive
plt.show()

print 'max torque gearshift:' , R.Sim.u_sol_CL.max()
print 'min torque gearshift:' , R.Sim.u_sol_CL.min()

############

CTC_controller.last_gear_i = 0
CTC_controller.n_gears = 1
R.R = [ np.diag([1,1,1]) ,   np.diag([1,1,1]) ]
R.computeSim( x_start , tf  , n , solver = 'euler' )

print 'max torque 1:1 :' , R.Sim.u_sol_CL.max()
print 'min torque 1:1 :' , R.Sim.u_sol_CL.min()

R.Sim.plot_CL('u')

############

CTC_controller.n_gears = 1
R.R = [ np.diag([10,10,10]) , np.diag([10,10,10]) ]
R.computeSim( x_start , tf  , n , solver = 'euler' )

print 'max torque 1:10 :' , R.Sim.u_sol_CL.max()
print 'min torque 1:10 :' , R.Sim.u_sol_CL.min()

R.Sim.plot_CL('u')