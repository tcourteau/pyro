# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 20:24:50 2016

@author: alex
"""

from AlexRobotics.dynamic import Manipulator    as M
from AlexRobotics.control import ComputedTorque as CTC

import matplotlib.pyplot as plt
import numpy as np


""" Define system """

# Define dynamic system
R  =  M.ThreeLinkManipulator()

# Define controller
CTC_controller      = CTC.SlidingModeController( R )
CTC_controller.lam  = 1
CTC_controller.D    = 10

# Goal
q_d                 = np.array([ np.pi, -np.pi * 0.25 , -np.pi * 0.25 ])
dq_d                = np.zeros( R.dof )
x_d                 = R.q2x( q_d , dq_d)
CTC_controller.goal =  x_d

# Asign feedback law to the dynamic system
R.ctl = CTC_controller.ctl


""" Simulation and plotting """

# Ploting a trajectory
x0   = [0,1,1,0,0,0]
tf   = 10
dt   = 0.001
n    = int( tf / dt ) + 1

R.plot3DAnimation( x0 , tf , n , solver = 'euler' )


R.Sim.plot_CL( 'x' )
R.Sim.plot_CL( 'u' )

# Hold figures alive
plt.show()