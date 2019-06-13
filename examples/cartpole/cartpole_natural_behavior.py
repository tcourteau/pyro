# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic  import cartpole
###############################################################################

sys  = cartpole.RotatingCartPole()

# Simultation
x_start = np.array([0,0.1,0,0])
sys.plot_phase_plane_trajectory( x_start  )
sys.sim.plot('xu')
sys.animate_simulation(1.0,True)