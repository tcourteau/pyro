# -*- coding: utf-8 -*-
"""
Created on Thu Jun  2 11:43:58 2016

@author: alex
"""

from AlexRobotics.dynamic  import Manipulator    as M
from AlexRobotics.control  import ComputedTorque as CTC

import matplotlib.pyplot as plt
import numpy             as np

""" Define system """

# Define dynamic system
R  =  M.OneLinkManipulator()
dt = 0.001

# Define controller
CTC_controller      = CTC.SlidingModeController( R )
CTC_controller.lam  = 1
CTC_controller.nab  = 1
CTC_controller.D    = 0

# Asign feedback law to the dynamic system
R.ctl = CTC_controller.ctl


""" Simulation and plotting """

# Ploting a trajectory
x_start = np.array([-3,0])
tf      = 10
n       = int( tf / dt ) + 1

R.plotAnimation( x_start , tf , n , solver = 'euler' )

# Time plot
R.Sim.plot_CL()

#PhasePlane Plot
R.Sim.phase_plane_trajectory( traj_CL=True, traj_OL=False, PP_CL=True, PP_OL=True )

# Hold figures alive
plt.show()