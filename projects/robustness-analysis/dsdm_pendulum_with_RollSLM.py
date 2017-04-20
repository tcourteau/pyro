# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.dynamic  import Prototypes             as Proto
from AlexRobotics.control  import RolloutComputedTorque  as RollCTC

import numpy as np


R_ctl  =  Proto.SingleRevoluteDSDM()
R      =  Proto.SingleRevoluteDSDM()

# Load mass
R.M                 = 0.2
R_ctl.M             = 0.3
R_ctl.ext_cst_force = 0


# Assign controller
Ctl               = RollCTC.RolloutSlidingModeController( R_ctl )

R.ctl              = Ctl.ctl

Ctl.n_gears       = 2
Ctl.w0            = 1.0
Ctl.lam           = 1.0
Ctl.nab           = 1.0
Ctl.D             = 2
Ctl.hysteresis    = True
Ctl.min_delay     = 0.5
Ctl.goal         = np.array([0,0])
Ctl.FixCtl.lam   = Ctl.lam 
Ctl.FixCtl.nab   = Ctl.nab 
Ctl.FixCtl.D     = Ctl.D
Ctl.horizon      = 0.5
Ctl.domain_check = False

# Max torque
R_ctl.u_ub = np.array([+10,500])
R_ctl.u_lb = np.array([-10,0])


""" Simulation and plotting """

# Ploting a trajectory
x_start = np.array([-6,0])
tf      = 10
dt      = 0.01
n       = int( tf / dt ) + 1


R.computeSim( x_start , tf , n , solver = 'euler' ) 

R.animateSim()
    
R.Sim.plot_CL()

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
