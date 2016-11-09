# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator     as HM
from AlexRobotics.planning import RandomTree             as RPRT
from AlexRobotics.control  import RminComputedTorque     as RminCTC
from AlexRobotics.control  import RolloutComputedTorque  as RollCTC

import numpy as np
import matplotlib.pyplot as plt

R_ctl  =  HM.HybridTwoLinkManipulator() # Controller model
R      =  HM.HybridTwoLinkManipulator() # Simulator


# Assign controller
Rollout     = RollCTC.RolloutSlidingModeController( R_ctl )

Rollout.goal         = np.array([0,0,0,0])
Rollout.FixCtl.lam   = 1.0
Rollout.FixCtl.D     = 2
Rollout.n_gears      = 4
Rollout.hysteresis   = True
Rollout.min_delay    = 0.2
Rollout.horizon      = 0.2


R.ctl              = Rollout.ctl



# Plot

tf = 10
x_start = np.array([-4,-2,1,1])

n  = int( np.round( tf / 0.01 ) ) + 1
R.plotAnimation( x_start , tf , n , solver = 'euler' )
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
#R.phase_plane_trajectory([0,0,3],x_start,tf,True,False,False,True)
