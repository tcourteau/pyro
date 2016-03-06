# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 12:54:37 2016

@author: alex
"""

from AlexRobotics.dynamic import simple1DOF    as RD1
from AlexRobotics.control import DPO           as DPO

import matplotlib.pyplot as plt

# Define dynamic system
pendulum  =  RD1.Pendulum()
m = 0.1
l = 1
g = 9.8
pendulum.setparams( m , g , l )


# Define controller
cost_function = 'quadratic'
ValueIterationAlgo     = DPO.ValueIteration1DOF( pendulum , cost_function )

ValueIterationAlgo.first_step()
ValueIterationAlgo.load_data( cost_function + 'pendulum' )
ValueIterationAlgo.compute_steps(100)
ValueIterationAlgo.save_data( cost_function + 'pendulum' ) 

# Plot Value Iteration Results
ValueIterationAlgo.plot_raw()

# Assign Controller
ValueIterationAlgo.assign_interpol_controller() 

# Ploting a trajectory
u_OL = [0]
x0   = [-6,-2]
tf   = 10
pendulum.phase_plane_trajectory( u_OL , x0 , tf , True , True , True, True )

# Time plot
pendulum.Sim.plot_OL()
pendulum.Sim.plot_CL()

# Hold figures alive
plt.show()