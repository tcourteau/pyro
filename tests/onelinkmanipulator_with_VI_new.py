# -*- coding: utf-8 -*-
"""
Created on Wed Jul 12 12:34:32 2017

@author: alxgr
"""

from AlexRobotics.dynamic import Manipulator    as M
from AlexRobotics.tools   import CostFunctions
from AlexRobotics.tools   import Discretizer
from AlexRobotics.control import ValueIteration as VI


""" Define system """

# Define dynamic system
R  =  M.OneLinkManipulator()


""" VI parameters """

# Discrete world 
dR = Discretizer.GridDynamicSystem2D( R )

# Cost Function
CF = CostFunctions.PureQuadratic( R.n , R.m )

Vi = VI.ValueIteration_2D( dR , CF )

Vi.initialize()

Vi.plot_J()

#Vi.compute_steps(50)
Vi.load_data()

Vi.plot_J()

Vi.assign_interpol_controller()

Vi.plot_policy(0)

Vi.save_data()

# Simulation and animation
x0   = [-4,2]
tf   = 10
R.plotAnimation( x0 , tf )
R.Sim.phase_plane_trajectory()