# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 20:24:50 2016

@author: alex
"""

from AlexRobotics.dynamic import Manipulator    as M
from AlexRobotics.control import ComputedTorque as CTC

import matplotlib.pyplot as plt
import numpy as np

# Define dynamic system
R  =  M.TwoLinkManipulator()

# Define controller
CTC_controller     = CTC.ComputedTorqueController( R )

# Asign feedback law to the dynamic system
R.ctl = CTC_controller.ctl

# Ploting a trajectory
x0   = [3,-1,0,0]
tf   = 10
R.plotAnimation( x0 , tf )

R.Sim.plot_CL( 'x' )
R.Sim.plot_CL( 'u' )

# Hold figures alive
plt.show()