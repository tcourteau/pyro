# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:05:08 2018

@author: Alexandre
"""
###############################################################################
import numpy as np
###############################################################################
from AlexRobotics.dynamic import pendulum
###############################################################################

sys = pendulum.DoublePendulum()

# Simultation
x_start  = np.array([-0.1,0,0,0])
sys.plot_trajectory( x_start , 10 , 10001, 'euler')
sys.sim.phase_plane_trajectory(0,2)
sys.animate_simulation()