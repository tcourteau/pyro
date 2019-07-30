# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 10:17:36 2018

@author: Alexandre
"""

###############################################################################
import numpy as np

###############################################################################
from pyro.dynamic  import integrator
    
###################################
# Simple integrator
###################################

ti = integrator.TripleIntegrator()

ti.ubar = np.array([1]) # constant input = 1

###################################
# Analysis
###################################

# Simulation
ti.plot_trajectory( np.array([2,0,0]) )
ti.sim.compute_cost()
ti.sim.plot('xu')
ti.sim.phase_plane_trajectory(0,1)