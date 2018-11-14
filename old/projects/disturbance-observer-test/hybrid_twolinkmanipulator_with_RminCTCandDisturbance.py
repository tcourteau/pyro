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

# Plant
R               =  HM.HybridTwoLinkManipulator()
R.f_dist_steady = np.array([ 10 , 10 ])

# Plant model for controller
R_hat           =  HM.HybridTwoLinkManipulator()

# Assign controller
CTC_controller     = RminCTC.RminComputedTorqueController( R_hat )
R.ctl              = CTC_controller.ctl

CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7

#CTC_controller.dist_obs_active = False
CTC_controller.dist_obs_active = True


# Plot

tf = 10
x_start = np.array([-4,-2,1,1])

n  = int( np.round( tf / 0.01 ) ) + 1
R.plotAnimation( x_start , tf , n , solver = 'euler' )
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
#R.phase_plane_trajectory([0,0,3],x_start,tf,True,False,False,True)
