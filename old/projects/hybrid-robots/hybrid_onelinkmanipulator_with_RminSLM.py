# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.control  import RminComputedTorque   as RminCTC

import numpy as np
import matplotlib.pyplot as plt

R  =  HM.HybridOneLinkManipulator()


# Assign controller
Ctl                = RminCTC.RminSlidingModeController( R )
#Ctl                = RminCTC.RminComputedTorqueController( R )
#Ctl               = RminCTC.RfixSlidingModeController( R , 1 )
R.ctl              = Ctl.ctl


Ctl.n_gears       = 2
Ctl.w0            = 1.0
Ctl.lam           = 1.0
Ctl.nab           = 1.0
Ctl.D             = 0

Ctl.hysteresis    = True
Ctl.min_delay     = 0.5


""" Simulation and plotting """

# Ploting a trajectory
x_start = np.array([-3,0])
tf      = 10
dt      = 0.001
n       = int( tf / dt ) + 1

R.plotAnimation( x_start , tf , n , solver = 'euler' )

# Time plot
R.Sim.plot_CL()

#PhasePlane Plot
R.Sim.phase_plane_trajectory( traj_CL=True, traj_OL=False, PP_CL=True, PP_OL=True )

# Hold figures alive
plt.show()