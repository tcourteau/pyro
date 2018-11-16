# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:05:08 2018

@author: Alexandre
"""
###############################################################################
import numpy as np
###############################################################################
from AlexRobotics.dynamic import pendulum
from AlexRobotics.control import nonlinear
###############################################################################

sys = pendulum.DoublePendulum()
ctl = nonlinear.ComputedTorqueController( sys )

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x_start  = np.array([2,1,0,0])
cl_sys.plot_trajectory( x_start  )
cl_sys.animate_simulation()