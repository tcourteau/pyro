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
sys = vehicle.KinematicBicyleModel()

# Set default wheel velocity and steering angle
sys.ubar = np.array([1,0.01])

# Plot open-loop behavior
sys.plot_trajectory( np.array([0,0,0]) , 1000 )

# Animate the simulation
sys.animate_simulation( 100 )