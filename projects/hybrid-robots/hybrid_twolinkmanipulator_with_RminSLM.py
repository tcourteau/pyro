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

R  =  HM.HybridTwoLinkManipulator()


# Assign controller
Ctl                = RminCTC.RminSlidingModeController( R )
#Ctl                = RminCTC.RminComputedTorqueController( R )
#Ctl               = RminCTC.RfixSlidingModeController( R , 3 )
R.ctl              = Ctl.ctl


Ctl.w0            = 1.0
Ctl.lam           = 1.0
Ctl.D             = 10.0

Ctl.hysteresis    = True
Ctl.min_delay     = 0.5



# Plot

tf = 10
x_start = np.array([-4,-2,1,1])

n  = int( np.round( tf / 0.05 ) ) + 1
R.plotAnimation( x_start , tf , n , solver = 'euler' )
R.Sim.plot_CL('x') 
R.Sim.plot_CL('u')
#R.phase_plane_trajectory([0,0,3],x_start,tf,True,False,False,True)
