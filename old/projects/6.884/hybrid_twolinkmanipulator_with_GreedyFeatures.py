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
R  =  HM.HybridTwoLinkManipulator()
R.u_lb = np.array([-5,-5, 0 ])
R.u_ub = np.array([ 5, 5, 3 ])

# Define controller
cost_function     = 'quadratic'
A                 = DPO.TD_Greedy_hybrid_2DOF_Features( R , cost_function )

A.W          = np.array([ 0.2 , 0.2 , 0.4 , 0.02 ])
#A.W          = np.array([ 1 , 0 , 0 , 0 ])
A.x0         = np.array([ -3, 1 ,  0 , 0 ])
A.max_error  = 0.5
A.eps        = 0.8
A.alpha      = 0.00001


#A.plot_J_hat()

A.training( 3 , random = True , show = False )

#A.W = np.array( [ 0.00596714 ,  0.05787924 , 0.1246888 , -0.00158788 ] )
#Weight = [ 0.09416771  0.20230782  0.37820584  0.01672458]

#A.plot_J_hat()

A.eps        = 1.0

R.plotAnimation( [-4,1,0,0] , tf = 12 , n = 241 ,  solver = 'euler' )#, save = True )
R.Sim.plot_CL('x')
R.Sim.plot_CL('u')
#R.Sim.plot_OL()
#R.Sim.phase_plane_trajectory()