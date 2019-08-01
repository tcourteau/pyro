# -*- coding: utf-8 -*-
"""
Created on Sun May 12 16:34:44 2019

@author: Alexandre
"""

###############################################################################
import numpy as np
###############################################################################
from pyro.control  import robotcontrollers
from pyro.dynamic  import manipulator
###############################################################################

torque_controlled_robot      = manipulator.TwoLinkManipulator()
torque_controlled_robot.ubar = np.array([2,0.8]) # constant inputs

x0        = np.array([0,1,0,0])

torque_controlled_robot.plot_animation( x0 )
torque_controlled_robot.sim.plot('xu')

speed_controlled_robot      = manipulator.SpeedControlledManipulator( 
                                             torque_controlled_robot )
speed_controlled_robot.ubar = np.array([2,0.8]) # constant inputs
    
x0        = np.array([0,1])
    
speed_controlled_robot.plot_animation( x0 )
speed_controlled_robot.sim.plot('xu')