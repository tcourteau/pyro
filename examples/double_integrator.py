# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 10:17:36 2018

@author: Alexandre
"""

###############################################################################
import numpy as np

###############################################################################
from pyro.dynamic  import integrator
from pyro.analysis import costfunction
    
###################################
# Simple integrator
###################################

di = integrator.DoubleIntegrator()

di.ubar = np.array([1]) # constant input = 1


###################################
# Analysis
###################################
    
# Phase plane behavior test
di.plot_phase_plane()

# Simulation
x0 = np.array([0,0])
di.plot_trajectory( x0 )
di.sim.plot('y')

# Cost computing
di.sim.compute_cost()
di.sim.plot('xuj')

# Time cost
di.sim.cf = costfunction.TimeCostFunction( di )
di.sim.compute_cost()
di.sim.plot('j')

# Phase plane trajectory
di.plot_phase_plane_trajectory( x0 )