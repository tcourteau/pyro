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
R.x_ub = np.array([ np.pi + 1 ,   np.pi  ])
R.x_lb = np.array([ np.pi - 1 , - np.pi  ])



# Set of params
r    = 10   # Gear ratio
R.d1 = 0   # Load side damping
R.Da = 0   # Actuator side damping


# Sweep of gear ratios, no damping
for r in xrange(1,10):
        
    R.ubar = np.array([ 0.0, r ])
    
    R.phase_plane()
    name = 'output/r' + str(r) + 'd' + str(R.Da)
    R.PP.phasefig.savefig( name , dpi=600 )
    
    
# Set of params
R.Da = 1   # Actuator side damping


# Sweep of gear ratios, with damping
for r in xrange(1,10):
        
    R.ubar = np.array([ 0.0, r ])
    
    R.phase_plane()
    name = 'output/r' + str(r) + 'd' + str(R.Da)
    R.PP.phasefig.savefig( name , dpi=600 )
    
# Set of params
R.Da = 2   # Actuator side damping

# Sweep of gear ratios, with damping
for r in xrange(1,10):
        
    R.ubar = np.array([ 0.0, r ])
    
    R.phase_plane()
    name = 'output/r' + str(r) + 'd' + str(R.Da)
    R.PP.phasefig.savefig( name , dpi=600 )
    