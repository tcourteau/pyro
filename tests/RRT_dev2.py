# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree    as RPRT
from AlexRobotics.dynamic  import DynamicSystem as RDDS
from AlexRobotics.dynamic  import Manipulator   as M

import numpy as np

R  =  M.TwoLinkManipulator()

x_start = np.array([-3,0,-3,0])

RRT = RPRT.RRT( R , x_start )

RRT.U = np.array([[10,0],[0,0],[-10,0],[0,10],[0,-10],[10,10],[-10,-10]])

RRT.compute_steps(10000,True)