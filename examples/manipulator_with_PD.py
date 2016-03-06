# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 20:24:50 2016

@author: alex
"""

from AlexRobotics.dynamic import Manipulator   as M
from AlexRobotics.control import linear        as RCL

import matplotlib.pyplot as plt
import numpy as np

# Define dynamic system
R  =  M.Manipulator()

# Define controller
kp = np.array([30,15])
kd = np.array([5,5])
PD_controller     = RCL.PD_nDOF( kp , kd )

# Asign feedback law to the dynamic system
R.ctl = PD_controller.u

# Ploting a trajectory
x0   = [-2,-2,0,0]
tf   = 10
R.plotAnimation( x0 , tf )

R.Sim.plot_CL( 'x' )
R.Sim.plot_CL( 'u' )

# Hold figures alive
plt.show()