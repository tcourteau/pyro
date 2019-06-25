# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:01:07 2018

@author: Alexandre
"""

###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic  import vehicle
###############################################################################


# Vehicule dynamical system
sys = vehicle.LateralDynamicBicycleModel()

# Set default steering angle and longitudinal velocity v_x
sys.ubar = np.array([0.2,15])

# Plot open-loop behavior
sys.plot_trajectory( np.array([0,0,0,0,0]) , 100 )


# Animate the simulation
sys.animate_simulation()