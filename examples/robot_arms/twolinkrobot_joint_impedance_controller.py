# -*- coding: utf-8 -*-
"""
Created on Sun May 12 16:34:44 2019

@author: Alexandre
"""

###############################################################################
import numpy as np
###############################################################################
from pyro.control  import robotcontrollers
from pyro.dynamic  import manipulator
###############################################################################

torque_controlled_robot      = manipulator.TwoLinkManipulator()

# Target
q_desired = np.array([0.5,0.5])

# Joint PID

dof = 2

joint_pid      = robotcontrollers.JointPID( dof )
joint_pid.rbar = q_desired
joint_pid.kp   = np.array([25, 5 ])
joint_pid.kd   = np.array([ 1, 0 ])


# Closed-loops

robot_with_joint_pid    = joint_pid    + torque_controlled_robot 

# Simulations

x0 = np.array([0,0,0,0])
tf = 5

robot_with_joint_pid.plot_animation( x0 , tf )
robot_with_joint_pid.sim.plot('xu')