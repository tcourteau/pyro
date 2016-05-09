# -*- coding: utf-8 -*-
"""
Created on Wed Mar 23 12:50:34 2016

@author: alex
"""

from AlexRobotics.dynamic import Manipulator          as M
from AlexRobotics.dynamic import Hybrid_Manipulator   as HM
from AlexRobotics.control import DPO_features         as DPO

import numpy as np

# Define dynamic system
R  =  HM.HybridOneLinkManipulator()
R.u_lb = np.array([-5,0])
R.u_ub = np.array([ 5,10])

# Define controller
cost_function     = 'time'
A                 = DPO.TD_Greedy_hybrid_1DOF_Features( R , cost_function )

A.W          = np.array([1,0.1,0])
A.x0         = np.array([-3,0,0,0])
A.max_error  = np.array([0.01,0.01])
A.alpha      = 0.00005


A.plot_J_hat()

A.training( 50 , random = True , show = False )

A.plot_J_hat()

R.plotAnimation( [ 4,0] , tf = 10 , n = 201 ,  solver = 'euler' )
R.Sim.plot_CL()
R.Sim.plot_OL()
R.Sim.phase_plane_trajectory()