# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.dynamic  import Prototypes             as Proto
from AlexRobotics.control  import RminComputedTorque     as RminCTC
from AlexRobotics.control  import RolloutComputedTorque  as RollCTC

import numpy as np


R_ctl  =  Proto.TwoPlanarSerialDSDM()
R      =  Proto.TwoPlanarSerialDSDM()

R.m2   = R_ctl.m2 + 0.3



# Assign controller
Ctl               = RollCTC.RolloutSlidingModeController( R_ctl )
#Ctl               = RminCTC.RminComputedTorqueController( R_ctl )

R.ctl              = Ctl.ctl

Ctl.n_gears       = 4
Ctl.w0            = 1.0
Ctl.lam           = 1.0
Ctl.nab           = 1.0
Ctl.D             = 1.0
Ctl.hysteresis    = True
Ctl.min_delay     = 0.5
Ctl.goal         = np.array([-2,+2,0,0])
Ctl.FixCtl.lam   = Ctl.lam 
Ctl.FixCtl.nab   = Ctl.nab 
Ctl.FixCtl.D     = Ctl.D
Ctl.horizon      = 0.5
Ctl.domain_check = False

# Max torque
R_ctl.u_ub = np.array([+10,+10,500])
R_ctl.u_lb = np.array([-10,-10,0])


""" Simulation and plotting """

# Ploting a trajectory
x_start = np.array([-4,-2,0,0])
tf      = 10
dt      = 0.01
n       = int( tf / dt ) + 1

R.computeSim( x_start , tf , n , solver = 'euler' ) 

R.animateSim()
R.Sim.plot_CL('x')    
R.Sim.plot_CL('u')

#
#Disturbances_Bounds = np.array([0,1,5,10])
#
#
#for D in Disturbances_Bounds:
#    
#    Ctl.FixCtl.D         = D
#    
#    Ctl.reset_hysteresis()
#    
#    R.computeSim( x_start , tf , n , solver = 'euler' ) 
#    
#    R.Sim.plot_CL()
#    
#    R.Sim.fig.canvas.set_window_title('D = ' + str(D) )
#    R.Sim.plots[3].set_ylim( 0 , 15 )
#    R.Sim.plots[3].set_yticks( [ 1 , 10 ] )
#
