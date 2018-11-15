# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 20:28:17 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.dynamic  import vehicle
from AlexRobotics.planning import discretizer
from AlexRobotics.analysis import costfunction
from AlexRobotics.planning import valueiteration
from AlexRobotics.control  import controller

sys  = vehicle.KinematicBicyleModel()

sys.x_ub = np.array( [+2,+2,+1] )
sys.x_lb = np.array( [-0,-0,-1] )

sys.u_ub = np.array( [+1,0] )
sys.u_lb = np.array( [-1,0] )

# Discrete world 
grid_sys = discretizer.GridDynamicSystem( sys , (21,21,3) , (3,3) , 0.1 ) 
# Cost Function
cf = costfunction.QuadraticCostFunction( sys )

cf.xbar = np.array( [1,1,0] ) # target
cf.INF  = 1E4
cf.EPS  = 0.2
cf.R    = np.array([[0.01,0],[0,0]])

# VI algo
vi = valueiteration.ValueIteration_3D( grid_sys , cf )

vi.uselookuptable = True
vi.initialize()
#vi.load_data('parking_vi')
vi.compute_steps(50)
vi.save_data('parking_vi')

vi.assign_interpol_controller()

vi.plot_J_ij( 1 )
vi.plot_policy_ij(0)
vi.plot_policy_ij(1)
#
cl_sys = controller.ClosedLoopSystem( sys , vi.ctl )
#
## Simulation and animation
x0   = [0.1,1,0]
tf   = 5

cl_sys.compute_trajectory( x0 , tf , 10001 , 'euler')
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()