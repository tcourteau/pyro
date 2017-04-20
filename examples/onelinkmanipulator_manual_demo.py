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
from AlexRobotics.planning import RandomTree
from AlexRobotics.control  import linear
from AlexRobotics.control  import ComputedTorque

###########################
# Create objects
###########################

Robot  =  Manipulator.OneLinkManipulator()

CTC    =  ComputedTorque.ComputedTorqueController( Robot )
RRT    =  RandomTree.RRT( Robot , x_start = [ -3.0 , 0.0 ] )
PD     =  linear.PD(  kp = 5 , kd = 2 ) 
PID    =  linear.PID( kp = 5  , kd = 2  , ki = 4 ) 


###########################
# Offline  Plannning
###########################

#RRT.find_path_to_goal( x_goal )
#RRT.plot_2D_Tree()

###########################
# Assign controller
###########################

#Robot.ctl             = RRT.trajectory_controller
#Robot.ctl             = CTC.ctl
#Robot.ctl             = PD.u
#Robot.ctl             = PID.u

###########################
# Simulation
###########################

Robot.plotAnimation( x0 = [ -3.0 , 0.0 ] , tf=20, n=20001, solver='euler' )

###########################
# Plots
###########################

#Robot.Sim.phase_plane_trajectory()
#Robot.Sim.plot_CL()


###########################
# and more
###########################

#from AlexRobotics.dynamic import CustomManipulator 
#BoeingArm = CustomManipulator.BoeingArm()
#BoeingArm.plot3DAnimation( x0 = np.array([0.2,0,0,0,0,0]) )




















###########################
# Objectives
###########################

x_start = np.array([-3.0,0])
x_goal  = np.array([0,0])


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
RRT.dyna_node_no_update   = 50
RRT.traj_ctl_kp           = 20
RRT.traj_ctl_kd           = 10

PID.dt                    = 0.001

CTC.w0                    = 2
