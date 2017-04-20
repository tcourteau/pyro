# -*- coding: utf-8 -*-
"""
Created on Sat Aug 27 13:15:41 2016

@author: agirard
"""


from AlexRobotics.dynamic  import Prototypes             as Proto
from AlexRobotics.control  import RminComputedTorque     as RminCTC
from AlexRobotics.control  import RolloutComputedTorque  as RollCTC

import numpy as np
import matplotlib
import matplotlib.pyplot as plt


""" Modes """

save_fig      = False
all_fig       = 'output/1link_xu.pdf'


""" Define dynamic system """

R      =  Proto.SingleRevoluteDSDM()
R.ubar =  np.array([0.0,474])

R.M             = 1
R.ext_cst_force = -0.1

""" Define control model """

R_ctl      =  Proto.SingleRevoluteDSDM()
R_ctl.ubar =  np.array([0.0,474])

R_ctl.M             = 1
R_ctl.ext_cst_force = 0

""" Define control problem """

x_start = np.array([-3.0,0])
x_goal  = np.array([0,0])



"""  CTC controller """

CTC_controller     = RminCTC.RminComputedTorqueController( R_ctl )

CTC_controller.goal         = x_goal
CTC_controller.w0           = 1.0
CTC_controller.zeta         = 0.7
CTC_controller.n_gears      = 2
#CTC_controller.traj_ref_pts = 'closest'
CTC_controller.traj_ref_pts = 'interpol'
CTC_controller.hysteresis   = False
CTC_controller.hys_level    = 0#1.0
CTC_controller.min_delay    = 0.2
CTC_controller.dist_obs_active = True

"""  Rollout Controller """

Rollout     = RollCTC.RolloutComputedTorqueController( R_ctl )

Rollout.goal         = x_goal
Rollout.w0           = 1.0
Rollout.zeta         = 0.7
Rollout.n_gears      = 2
Rollout.hysteresis   = True
Rollout.min_delay    = 0.2
Rollout.horizon      = 0.3
Rollout.dist_obs_active = True

"""  Assign controller """

#R.ctl              = CTC_controller.ctl
R.ctl              = Rollout.ctl

""" Simulation """

tf = 10
dt = 0.01

R.computeSim( x_start , tf , n = int( tf / dt  ) + 1 , solver = 'euler' ) 

R.animateSim()


""" Plot """

R.Sim.fontsize = 7
t_ticks = [0,2,4,6,8,10]

R.Sim.plot_CL()

R.Sim.plots[0].set_yticks( [-4,0] )
R.Sim.plots[1].set_yticks( [-4,0, 4] )
#R.Sim.plots[2].set_yticks( [-8,0,8] )
R.Sim.plots[3].set_ylim(    0,500 )
R.Sim.plots[3].set_yticks( [23.2,474] )
R.Sim.plots[3].set_xlim(    0,10 )
R.Sim.plots[3].set_xticks( t_ticks )
R.Sim.fig.set_size_inches(4,2.5)
R.Sim.fig.canvas.draw()


if save_fig:
    R.Sim.fig.savefig( all_fig , format='pdf', bbox_inches='tight', pad_inches=0.05)


