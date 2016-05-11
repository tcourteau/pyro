# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 14:45:38 2016

@author: alex
"""

from AlexRobotics.dynamic import Manipulator   as M
from AlexRobotics.control import DPO           as DPO

import matplotlib.pyplot as plt

# Define dynamic system
R  =  M.OneLinkManipulator()

# Define controller
cost_function = 'quadratic'
ValueIterationAlgo     = DPO.ValueIteration1DOF( R , cost_function )

path = 'data/'
name = path + 'R1' + cost_function 

n_steps = 1
#n_steps = 200

ValueIterationAlgo.first_step()
ValueIterationAlgo.load_data( name )
ValueIterationAlgo.compute_steps( n_steps )
ValueIterationAlgo.save_data( name ) 

# Plot Value Iteration Results
ValueIterationAlgo.plot_raw()
ValueIterationAlgo.plot_J_nice( 1.5 )


# Assign Controller
ValueIterationAlgo.assign_interpol_controller() 

# Ploting a trajectory
u_OL = [0]
x0   = [3,0]
tf   = 10
R.plotAnimation( x0 , tf )
R.phase_plane_trajectory( u_OL , x0 , tf , True , True , True, True )

# Time plot
R.Sim.plot_CL()

# Hold figures alive
plt.show()