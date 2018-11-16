# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:01:07 2018

@author: Alexandre
"""

###############################################################################
import numpy as np

###############################################################################
from AlexRobotics.dynamic  import vehicle


###############################################################################

sys = vehicle.KinematicBicyleModel()
    
sys.ubar = np.array([1,0.01])
sys.plot_trajectory( np.array([0,0,0]) , 1000 )

sys.animate_simulation( 100 )