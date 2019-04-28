# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

import numpy as np

from pyro.dynamic  import vehicle
from pyro.planning import discretizer
from pyro.analysis import costfunction
from pyro.planning import valueiteration

sys  = vehicle.HolonomicMobileRobotwithObstacles()

# Discrete world 
grid_sys = discretizer.GridDynamicSystem( sys , (51,51) , (3,3) ) 

# Cost Function
cf = costfunction.QuadraticCostFunction( sys )

cf.INF = 1E9

# VI algo
vi = valueiteration.ValueIteration_2D( grid_sys , cf )

vi.initialize()
vi.load_data('holonomic_vi')
vi.compute_steps(1)
vi.plot_cost2go(40000)
vi.assign_interpol_controller()
vi.plot_policy(0)
vi.plot_policy(1)
#vi.save_data('holonomic_vi')

# Closed loop
cl_sys = vi.ctl + sys

# Simulation and animation
x0   = [9,0]
tf   = 20
cl_sys.plot_trajectory( x0 , tf )
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()