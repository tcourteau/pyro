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
save_fig      = False
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

t = R.Sim.t

t1 = R.Sim.u_sol_CL[:,0]
t2 = R.Sim.u_sol_CL[:,1]
t3 = R.Sim.u_sol_CL[:,2]

l = R.Sim.n
r1 = np.zeros( n )
r2 = np.zeros( n )
r3 = np.zeros( n )

for i in xrange(l):
    
    index = int( R.Sim.u_sol_CL[i,3] )

    r1[i] = R.R[ index ][0,0]
    r2[i] = R.R[ index ][1,1]
    r3[i] = R.R[ index ][2,2]


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
    #R.animate3DSim( 1.0 , True ,  'output/' + test_name )
    
else:
    
    R.animate3DSim()
    #pass


R.lw = 2.5
R.show_traj_3D([180,310,350,400,500])
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
R.ax_show_3D.text(2.2, 1, 1, 'Configuration A' )
R.ax_show_3D.text(0, 0.8, 2.5, 'Configuration B')
R.fig_show_3D.canvas.draw()

if save_fig:

    R.fig_show_3D.savefig( 'output/' + '3d_traj_'+test_name+ '.pdf' , format = 'pdf' , bbox_inches='tight', pad_inches=0.05 )

# Hold figures alive
plt.show()


# Compute integral cost
R.Sim.Q = np.diag([0,0,0,0,0,0])
R.Sim.R = np.diag([1,1,1,0])

R.Sim.compute()

print 'max torque gearshift:' , R.Sim.u_sol_CL.max()
print 'min torque gearshift:' , R.Sim.u_sol_CL.min()
#print '      cost gearshift:' , R.Sim.J

#############
#
#CTC_controller.last_gear_i = 0
#CTC_controller.n_gears = 1
#R.R = [ np.diag([1,1,1]) ,   np.diag([1,1,1]) ]
#
#R.Sim.compute()
#
#print 'max torque 1:1 :' , R.Sim.u_sol_CL.max()
#print 'min torque 1:1 :' , R.Sim.u_sol_CL.min()
#print '      cost 1:1 :' , R.Sim.J
#
#R.Sim.plot_CL('u')
#
#############
#
#CTC_controller.n_gears = 1
#R.R = [ np.diag([10,10,10]) , np.diag([10,10,10]) ]
#
#R.Sim.compute()
#
#print 'max torque 1:10 :' , R.Sim.u_sol_CL.max()
#print 'min torque 1:10 :' , R.Sim.u_sol_CL.min()
#print '      cost 1:10 :' , R.Sim.J
#
#R.Sim.plot_CL('u')




def plot_3d():

    fontsize = 7
    
    matplotlib.rc('xtick', labelsize=fontsize )
    matplotlib.rc('ytick', labelsize=fontsize )
    
    
    simfig , plot = plt.subplots(2, sharex=True,figsize=(4, 2),dpi=300, frameon=True)
    
    simfig.canvas.set_window_title('Closed loop trajectory')
    
    plot[0].plot( t ,  t1 , 'r',  label = 'DoF 1')
    plot[0].plot( t ,  t2 , 'b--' ,  label = 'DoF 2')
    plot[0].plot( t ,  t3 , 'g-.'  ,  label = 'DoF 3')
    plot[0].set_yticks([-10,0,10])
    plot[0].set_ylim(-15,15)
    plot[0].grid(True)
    plot[0].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
    #legend.get_frame().set_alpha(0.4)
    
    plot[1].plot( t ,  r1 , 'r',  label = 'DoF 1')
    plot[1].plot( t ,  r2 , 'b--',  label = 'DoF 2')
    plot[1].plot( t ,  r3 , 'g-.',  label = 'DoF 3')
    plot[1].set_yticks([1,10])
    plot[1].set_ylim(0,11)
    plot[1].grid(True)
    plot[1].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
    
    plot[1].set_ylabel('Ratio' , fontsize=fontsize )
    plot[0].set_ylabel('Torque \n [Nm]' , fontsize=fontsize )
    
    plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
    #plot[-1].set_xlim(0,4)
    
    #plot[-1].set_xticks([0.65,1.16,1.39,2.63])
    #plot[-1].set_xticks([0,1,2,3,4])

    simfig.tight_layout()
    
    simfig.show()
    
    return simfig
    
    
fig = plot_3d()
fig.savefig( 'output/' + '3du.pdf' , format='pdf', bbox_inches='tight', pad_inches=0.05)