# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 12:54:37 2016

@author: alex
"""

from AlexRobotics.dynamic import simple1DOF    as RD1
from AlexRobotics.control import DPO           as DPO

import matplotlib.pyplot as plt

""" Define system """

# Define dynamic system
pendulum  =  RD1.Pendulum()
m = 0.5
l = 1
g = 9.8
pendulum.setparams( m , g , l )

# Define controller
cost_function = 'quadratic'
ValueIterationAlgo     = DPO.ValueIteration1DOF( pendulum , cost_function )

path = 'data/'

ValueIterationAlgo.first_step()
ValueIterationAlgo.load_data( path + cost_function + 'pendulum' )
ValueIterationAlgo.compute_steps(1)
ValueIterationAlgo.save_data( path + cost_function + 'pendulum' ) 

# Plot Value Iteration Results
ValueIterationAlgo.plot_raw()

# Assign Controller
ValueIterationAlgo.assign_interpol_controller() 

""" Simulation and plotting """

# Ploting a trajectory
u_OL = [0]
x0   = [-6,-2]
tf   = 25
pendulum.phase_plane_trajectory( u_OL , x0 , tf , True , True , True, True )

# Time plot
pendulum.Sim.plot_CL()

# Hold figures alive
plt.show()