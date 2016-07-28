# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 16:52:05 2016

@author: agirard
"""

from AlexRobotics.dynamic import Manipulator   as M

import numpy as np


R2  =  M.TwoLinkManipulator()

R3  =  M.ThreeLinkManipulator()
R3.axis_to_plot = [0,2] # x-z plane


# Equivalent initial conditions
R2.plotAnimation([0.1,0,0,0])
R3.plotAnimation([0,0.1-np.pi*0.5,0,0,0,0])


# 3D plot same intial condition
R3_b  =  M.ThreeLinkManipulator()
R3_b.plot3DAnimation([0,0.1-np.pi*0.5,0,0,0,0])



# 3D plot 3d-motions 
R3_c  =  M.ThreeLinkManipulator()
R3_c.ubar = np.array([3,0,0]) # Torque arround z-axis
R3_c.plot3DAnimation([0,0.1-np.pi*0.5,0,0,0,0])
