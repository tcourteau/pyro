# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 10:13:40 2018

@author: Alexandre
"""
###############################################################################
import numpy as np

###############################################################################
from pyro.dynamic  import integrator
    
###################################
# Simple integrator
###################################

si = integrator.SimpleIntegrator()

si.ubar = np.array([1]) # constant input = 1

###################################
# Analysis
###################################

# Phase plane
si.plot_phase_plane(0,0) # only one state for two axis!

# Simulation
si.compute_trajectory( np.array([2]) )

# Compute cost
si.sim.compute_cost()

# Plot output
si.sim.plot('xuj')
si.sim.phase_plane_trajectory(0,0)