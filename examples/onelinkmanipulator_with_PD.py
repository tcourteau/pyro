# -*- coding: utf-8 -*-
"""
Created on Thu Jun  2 11:43:58 2016

@author: alex
"""

from AlexRobotics.dynamic  import Manipulator   as M
from AlexRobotics.control  import linear        as RCL

import matplotlib.pyplot as plt
import numpy             as np

""" Define system """

# Define dynamic system
R  =  M.OneLinkManipulator()

# Define controller
kp = 20
kd = 10
PD_controller     = RCL.PD( kp , kd )

# Asign feedback law to the dynamic system
R.ctl = PD_controller.u


""" Simulation and plotting """

# Ploting a trajectory
x_start = np.array([-3,0])
tf      = 8
n       = int( tf / 0.05 ) + 1

R.plotAnimation( x_start , tf , n , solver = 'ode' )

# Time plot
#R.Sim.plot_OL()
R.Sim.plot_CL()

#PhasePlane Plot
R.Sim.phase_plane_trajectory( traj_CL=True, traj_OL=False, PP_CL=True, PP_OL=True )

# Hold figures alive
plt.show()