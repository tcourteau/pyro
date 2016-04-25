# -*- coding: utf-8 -*-
"""
Created on Wed Mar 23 12:50:34 2016

@author: alex
"""

from AlexRobotics.dynamic import Manipulator   as M
from AlexRobotics.control import DPO_features  as DPO

import numpy as np

# Define dynamic system
R  =  M.OneLinkManipulator()
R.u_lb = np.array([-5])
R.u_ub = np.array([ 5])

# Define controller
cost_function     = 'time'
A                 = DPO.TD_Greedy_1DOF_Features( R , cost_function )

A.W = np.array([1,0.1,0])


A.plot_J_hat()

A.training( 100 , True , False )

A.plot_J_hat()

R.plotAnimation( A.x0 , tf = 25 , n = 201 ,  solver = 'euler' )

R.Sim.plot_CL()