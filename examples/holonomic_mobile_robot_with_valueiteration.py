# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.dynamic import vehicle
from AlexRobotics.planning import discretizer
from AlexRobotics.analysis import costfunction
from AlexRobotics.planning import valueiteration
from AlexRobotics.control import controller

sys  = vehicle.HolonomicMobileRobotwithObstacles()

# Discrete world 
grid_sys = discretizer.GridDynamicSystem2D( sys ) # TODO

# Cost Function
cf = costfunction.QuadraticCostFunction( sys )

vi = valueiteration.ValueIteration_2D( grid_sys , cf )

vi.initialize()

vi.plot_J()

vi.load_data()
vi.compute_steps(50)
#vi.load_data()

vi.plot_J()

vi.assign_interpol_controller()

vi.plot_policy(0)

vi.save_data()

cl_sys = controller.ClosedLoopSystem( sys , vi.ctl )

# Simulation and animation
x0   = [8,0]
tf   = 20
cl_sys.plot_trajectory_CL( x0 , tf )
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()