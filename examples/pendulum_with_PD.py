# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 19:03:23 2016

@author: alex
"""

from AlexRobotics.dynamic import simple1DOF    as RD1
from AlexRobotics.control import linear        as RCL

import matplotlib.pyplot as plt

""" Define system """

# Define dynamic system
pendulum  =  RD1.Pendulum()
m = 1
l = 1
g = 9.8
pendulum.setparams( m , g , l )

# Define controller
kp = 2
kd = 2
PD_controller     = RCL.PD( kp , kd )

# Asign feedback law to the dynamic system
pendulum.ctl = PD_controller.ctl


""" Simulation and plotting """

# Ploting a trajectory
x0   = [-2,-2]
tf   = 10
pendulum.phase_plane_trajectory( x0 , tf , True , True , True, True )

# Time plot
pendulum.Sim.plot_OL()
pendulum.Sim.plot_CL()


# Hold figures alive
plt.show()