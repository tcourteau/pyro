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
from pyro.analysis import simulation
###############################################################################

sys  = pendulum.SinglePendulum()

###############################################################################

# Planning

traj   = plan.load_trajectory('pendulum_rrt.npy')
q_goal = np.array([-3.14])

###############################################################################

# P
kp = 5
kd = 0
ki = 0
p_ctl      = linear.JointPID(sys, kp , ki, kd)
p_ctl.rbar = q_goal

# PD
kp = 5
kd = 2
ki = 0
pd_ctl      = linear.JointPID(sys, kp , ki, kd)
pd_ctl.rbar = q_goal

# PID
kp = 5
kd = 2
ki = 1
pid_ctl      = linear.JointPID(sys, kp , ki, kd)
pid_ctl.rbar = q_goal

# Computed Torque
ctc_ctl      = nonlinear.ComputedTorqueController( sys )
ctc_ctl.rbar = q_goal
ctc_ctl.w0   = 2.0
ctc_ctl.zeta = 0.8 

# Sliding Mode 
sld_ctl      = nonlinear.SlidingModeController( sys )
sld_ctl.lam  = 1
sld_ctl.gain = 5
sld_ctl.rbar = q_goal

# OpenLoop with traj
traj_ctl = plan.OpenLoopController( traj )

# Computed Torque with traj
traj_ctc_ctl      = nonlinear.ComputedTorqueController( sys , traj )
traj_ctc_ctl.rbar = q_goal
traj_ctc_ctl.w0   = 2.0
traj_ctc_ctl.zeta = 0.8 

# Sliding Mode with traj
traj_sld_ctl      = nonlinear.SlidingModeController( sys , traj )
traj_sld_ctl.lam  = 1
traj_sld_ctl.gain = 5
traj_sld_ctl.rbar = q_goal

###############################################################################

# Controller selection

#ctl = p_ctl
#ctl = pd_ctl
#ctl = pid_ctl
#ctl = ctc_ctl
#ctl = sld_ctl
#ctl = traj_ctl
ctl = traj_ctc_ctl 
#ctl = traj_sld_ctl


###############################################################################

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
q0 = 0
tf = 10
cl_sys.sim = simulation.CLosedLoopSimulation( cl_sys , tf , tf * 1000 + 1 , 'euler' )
cl_sys.sim.x0 = np.array([q0,0])
cl_sys.sim.compute()
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()
cl_sys.sim.phase_plane_trajectory(0,1)