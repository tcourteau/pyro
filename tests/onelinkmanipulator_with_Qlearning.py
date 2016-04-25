# -*- coding: utf-8 -*-
"""
Created on Wed Mar 23 12:50:34 2016

@author: alex
"""

from AlexRobotics.dynamic import Manipulator   as M
from AlexRobotics.control import DPO           as DPO

import numpy as np
import matplotlib.pyplot as plt

# Define dynamic system
R  =  M.OneLinkManipulator()

# Define controller
cost_function     = 'quadratic'
QLearningAlgo     = DPO.QLearning1DOF( R , cost_function )

QLearningAlgo.first_step()

x0 = np.array( [ -3 , 0 ] )
QLearningAlgo.x0 = x0

QLearningAlgo.training( 1 , True , True )

QLearningAlgo.plot_J()

R.plotAnimation( x0 )