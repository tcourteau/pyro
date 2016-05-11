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


path = 'data/'
name = 'R1_Qlearning' + cost_function 


QLearningAlgo.first_step()
QLearningAlgo.load_data( name )

x0 = np.array( [ -3 , 0 ] )
QLearningAlgo.x0 = x0

QLearningAlgo.training( 1 , False , False )


#QLearningAlgo.plot_J()
QLearningAlgo.plot_raw_nice(3)
#R.plotAnimation( x0 )
#R.Sim.plot_CL()
#
#tf   = 10
#R.plotAnimation( x0 , tf , n = 201 ,  solver = 'ode' , save = True , file_name = name  )