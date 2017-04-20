# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 20:24:50 2016

@author: alex
"""

from AlexRobotics.dynamic import Manipulator    as M
from AlexRobotics.control import ComputedTorque as CTC

import matplotlib.pyplot as plt


""" Define system """

# Define dynamic system
R  =  M.TwoLinkManipulator()

# Define controller
CTC_controller      = CTC.SlidingModeController( R )
CTC_controller.lam  = 1
CTC_controller.nab  = 1
CTC_controller.D    = 0

# Asign feedback law to the dynamic system
R.ctl = CTC_controller.ctl


""" Simulation and plotting """

# Ploting a trajectory
x0   = [3,-1,0,0]
tf   = 10
dt   = 0.01
n    = int( tf / dt ) + 1

R.plotAnimation( x0 , tf , n , solver = 'euler' )

R.Sim.plot_CL( 'x' )
R.Sim.plot_CL( 'u' )

# Hold figures alive
plt.show()