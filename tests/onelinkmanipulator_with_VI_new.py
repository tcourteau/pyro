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

Vi.compute_steps(200)

Vi.plot_J()