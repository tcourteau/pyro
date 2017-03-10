# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 14:45:38 2016

@author: alex
"""

#from AlexRobotics.dynamic import Manipulator   as M
from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import DPO           as DPO

import matplotlib.pyplot as plt

""" Define system """

# Define dynamic system
R  =  HM.HybridOneLinkManipulator()

# Define controller
cost_function      = 'quadratic'
ValueIterationAlgo = DPO.ValueIteration_hybrid_1DOF( R , cost_function )

path = 'data/'

ValueIterationAlgo.first_step()
ValueIterationAlgo.load_data( path + 'R1H' + cost_function )
#ValueIterationAlgo.compute_steps(50)
ValueIterationAlgo.compute_steps(1)
ValueIterationAlgo.save_data( path + 'R1H' + cost_function ) 

# Plot Value Iteration Results
#ValueIterationAlgo.plot_raw()
#ValueIterationAlgo.plot_J_nice( 2 )
ValueIterationAlgo.plot_raw_nice( 2 )

# Assign Controller
ValueIterationAlgo.assign_interpol_controller() 

""" Simulation and plotting """

# Simulation and animation
x0   = [-3,0]
tf   = 10
R.plotAnimation( x0 , tf )

# Phase plane plot
R.Sim.phase_plane_trajectory( True , True , True, True )

# Time plot
R.Sim.plot_CL()

# Hold figures alive
plt.show()