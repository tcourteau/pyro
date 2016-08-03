# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree    as RPRT
from AlexRobotics.dynamic  import Manipulator   as M

import numpy as np
import matplotlib.pyplot as plt

""" Define system """

R  =  M.ThreeLinkManipulator()

x_start = np.array([0,   np.pi * 0.5 ,0,0,0,0])
x_goal  = np.array([0, - np.pi * 0.5 ,0,0,0,0])

RRT = RPRT.RRT( R , x_start )

T = 12 # torque

RRT.U = np.array([[0,T,0],[0,0,0],[0,-T,0],[0,0,T],[0,0,-T],[0,T,T],[0,-T,-T],[0,-T,T],[0,T,-T]])

RRT.dt                    = 0.1
RRT.goal_radius           = 2.0
RRT.max_nodes             = 15000
RRT.max_solution_time     = 10

#RRT.compute_steps(1000,True)
RRT.find_path_to_goal( x_goal )