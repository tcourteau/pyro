# -*- coding: utf-8 -*-
"""
Created on Sat Jul 30 13:22:19 2016

@author: agirard
"""

from AlexRobotics.dynamic import Manipulator   as M
from AlexRobotics.control import linear        as RCL

import matplotlib.pyplot as plt
import numpy as np

""" Define system """

# Define dynamic system
R  =  M.ThreeLinkManipulator()


# Define controller
kp  = np.array([10,60,30])
kd  = np.array([5,15,10])
q_d = np.array([ np.pi, -np.pi * 0.25 , -np.pi * 0.25 ])
PD_controller     = RCL.PD_nDOF( kp , kd , q_d )

# Asign feedback law to the dynamic system
R.ctl = PD_controller.u


""" Simulation and plotting """

# Show target config
R.show_3D( q_d )

# Ploting a trajectory
x0   = [0,0,0,0,0,0]
tf   = 10
R.plot3DAnimation( x0 , tf )

R.Sim.plot_CL( 'x' )
R.Sim.plot_CL( 'u' )

# Hold figures alive
plt.show()