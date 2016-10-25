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

# Define real dynamic system
R               =  M.TwoLinkManipulator()
R.f_dist_steady = np.array([ 44.78 , 0 ])

# Define approx dynamic system used by controller
R_hat              =  M.TwoLinkManipulator()
#R_hat.f_dist_steady    = np.array([ 0 , 0 ]) # Model not aware of disturbance

# Define controller
CTC_controller                 = CTC.ComputedTorqueController( R_hat )
CTC_controller.w0              = 1

#CTC_controller.dist_obs_active = False
CTC_controller.dist_obs_active = True

# Asign feedback law to the dynamic system
R.ctl = CTC_controller.ctl


""" Simulation and plotting """

# Ploting a trajectory
x0   = [3,-1,0,0]
tf   = 10
R.plotAnimation( x0 , tf , n = 1001 ,  solver='euler' )

R.Sim.plot_CL( 'x' )
R.Sim.plot_CL( 'u' )

# Hold figures alive
plt.show()