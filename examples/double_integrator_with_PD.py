# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 16:01:32 2016

@author: alex
"""

from AlexRobotics.dynamic import DynamicSystem as RDDS
from AlexRobotics.control import linear        as RCL

import matplotlib.pyplot as plt
import numpy as np

""" Define system """

# Define dynamic system
double_integrator = RDDS.DynamicSystem()

# Define controller
kp = 1
kd = 1
PD_controller     = RCL.PD( kp , kd )

# Asign feedback law to the dynamic system
double_integrator.ctl = PD_controller.u

""" Simulation and plotting """

# Phase plot
#PP = RDDS.PhasePlot( double_integrator )
#PP.compute()
#PP.plot()

open_loop_vector_field   = True
closed_loop_vector_field = True

double_integrator.phase_plane( closed_loop_vector_field , open_loop_vector_field )

# Trajectory simulation and plotting
double_integrator.ubar   = np.array([3])
x0                       = np.array([-5,-5])
tf                       = 10

open_loop_trajectory     = True
closed_loop_trajectory   = True

double_integrator.phase_plane_trajectory( x0 , tf , True , True , True, True )

# Time plot
double_integrator.Sim.plot_OL()
double_integrator.Sim.plot_CL()

# Hold figures alive
plt.show()
