# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:05:08 2018

@author: Alexandre
"""
###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic  import pendulum
from pyro.control  import linear
from pyro.control  import nonlinear
from pyro.planning import plan
from pyro.planning import randomtree
###############################################################################

sys  = pendulum.SinglePendulum()

###############################################################################

# Planning

traj   = plan.load_trajectory('pendulum_rrt.npy')
q_goal = np.array([3.14])

###############################################################################




# Computed Torque
ctc_ctl      = nonlinear.ComputedTorqueController( sys )
ctc_ctl.rbar = q_goal

# Sliding Mode 
sld_ctl      = nonlinear.SlidingModeController( sys )
sld_ctl.lam  = 5
sld_ctl.gain = 2
sld_ctl.rbar = q_goal

# OpenLoop with traj
traj_ctl = plan.OpenLoopController( traj )

# Computed Torque with traj
traj_ctc_ctl      = nonlinear.ComputedTorqueController( sys , traj )
traj_ctc_ctl.rbar = q_goal

# Sliding Mode with traj
traj_sld_ctl      = nonlinear.SlidingModeController( sys , traj )
traj_sld_ctl.lam  = 5
traj_sld_ctl.gain = 2
traj_sld_ctl.rbar = q_goal

###############################################################################

# Controller selection

ctl = 
ctl = 
ctl = 
ctl = ctc_ctl
ctl = sld_ctl
ctl = traj_ctl
ctl = traj_ctc_ctl 
ctl = traj_sld_ctl


###############################################################################

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x_start  = np.array([0.1,0])
cl_sys.plot_trajectory(x_start, 5, 10001, 'euler')
cl_sys.animate_simulation()