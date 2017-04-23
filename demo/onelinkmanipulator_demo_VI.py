# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

import numpy as np

###########################
# Load libs
###########################

from AlexRobotics.dynamic  import Manipulator 
from AlexRobotics.control  import linear
from AlexRobotics.control  import ComputedTorque
from AlexRobotics.planning import RandomTree
from AlexRobotics.control  import DPO


###########################
# Objectives
###########################

x_start = np.array([-3.0, 0.0])
x_goal  = np.array([ 0.0, 0.0])


###########################
# Create objects
###########################

Robot  =  Manipulator.OneLinkManipulator()

PD     =  linear.PD(  kp = 5 , kd = 2 ) 
PID    =  linear.PID( kp = 5  , kd = 2  , ki = 4 )
CTC    =  ComputedTorque.ComputedTorqueController( Robot )
SLD    =  ComputedTorque.SlidingModeController( Robot )
RRT    =  RandomTree.RRT( Robot , x_start )
VI     =  DPO.ValueIteration1DOF( Robot , 'quadratic' )


############################
# Params
############################

tmax = 8                            # max motor torque
Robot.u_ub = np.array([ tmax])      # Control Upper Bounds
Robot.u_lb = np.array([-tmax])      # Control Lower Bounds
RRT.x_start               = x_start
RRT.discretizeactions( 3 )
RRT.dt                    = 0.1
RRT.goal_radius           = 0.3
RRT.max_nodes             = 5000
RRT.max_solution_time     = 5
RRT.dyna_plot             = True
RRT.dyna_node_no_update   = 10
RRT.traj_ctl_kp           = 25
RRT.traj_ctl_kd           = 10
PID.dt                    = 0.001
CTC.w0                    = 2
SLD.lam                   = 1
SLD.nab                   = 0
SLD.D                     = 5


###########################
# Offline  Plannning
###########################

#RRT.find_path_to_goal( x_goal )
#RRT.plot_2D_Tree()


###########################
# Offline  Optimization
###########################

VI.first_step()
VI.load_data( 'data/' + 'R1' + 'quadratic' )
VI.compute_steps(1)
#
## Plot Value Iteration Results
#ValueIterationAlgo.plot_raw()
#ValueIterationAlgo.plot_J_nice( 2 )

###########################
# Assign controller
###########################

#Robot.ctl             = PD.ctl
#Robot.ctl             = PID.ctl
#Robot.ctl             = CTC.ctl
#Robot.ctl             = SLD.ctl
#Robot.ctl             = RRT.trajectory_controller

VI.assign_interpol_controller() 


###########################
# Simulation
###########################

Robot.plotAnimation( x_start , tf=10, n=10001, solver='euler' )

###########################
# Plots
###########################

Robot.Sim.phase_plane_trajectory()
Robot.Sim.phase_plane_trajectory( PP_OL = False , PP_CL = True )
Robot.Sim.plot_CL()


###########################
# and more
###########################

#from AlexRobotics.dynamic import CustomManipulator 
#BoeingArm = CustomManipulator.BoeingArm()
#BoeingArm.plot3DAnimation( x0 = np.array([0.2,0,0,0,0,0]) )



