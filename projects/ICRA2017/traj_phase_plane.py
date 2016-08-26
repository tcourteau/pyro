# -*- coding: utf-8 -*-
"""
Created on Sun Aug  7 15:28:12 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM

import matplotlib.pyplot as plt
import numpy             as np

""" Define system """

# Define dynamic system
R  =  HM.HybridOneLinkManipulator()

# Definre domain of interest
R.x_ub = np.array([ 2* np.pi  ,   2*np.pi  ])
R.x_lb = np.array([ - 2* np.pi , -2* np.pi  ])



# Set of params
r    = 1   # Gear ratio
R.d1 = 0   # Load side damping
R.Da = 0   # Actuator side damping
R.I1 = 1
R.Ia = 0

R.ubar = np.array([ 0.0, r ])

R.phase_plane()
name = 'output/r' + str(r) + 'd' + str(R.Da)
#R.PP.phasefig.savefig( name , dpi=600 )
    
    
# Set of params
R.Da = 1   # Actuator side damping

