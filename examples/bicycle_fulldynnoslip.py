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
sys = vehicle.FullDynamicWithoutSlipBicycleModel()

# Set default steering angle and longitudinal velocity v_x
sys.ubar = np.array([0.5,0,1000])

# Plot open-loop behavior
sys.plot_trajectory( np.array([1,0,0,0,0,0]) , 50 )


# Animate the simulation
sys.animate_simulation()