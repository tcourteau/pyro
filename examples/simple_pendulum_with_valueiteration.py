# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.dynamic import pendulum
from AlexRobotics.planning import discretizer
from AlexRobotics.analysis import costfunction
from AlexRobotics.planning import valueiteration
from AlexRobotics.control import controller

sys  = pendulum.SinglePendulum()

# Discrete world 
grid_sys = discretizer.GridDynamicSystem2D( sys )

# Cost Function
cf = costfunction.QuadraticCostFunction( sys )

cf.xbar = np.array([ -3.14 , 0 ])
cf.INF  = 10000

vi = valueiteration.ValueIteration_2D( grid_sys , cf )

vi.initialize()

vi.plot_J()

vi.load_data()
vi.compute_steps(500)
#vi.load_data()

vi.plot_J()

vi.assign_interpol_controller()

vi.plot_policy(0)

vi.save_data()

cl_sys = controller.ClosedLoopSystem( sys , vi.ctl )

# Simulation and animation
x0   = [0,0]
tf   = 10
cl_sys.plot_trajectory_CL( x0 , tf )
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()