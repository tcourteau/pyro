# -*- coding: utf-8 -*-
"""
Created on Wed Sep 4 11:01:39 2019

@author: Thomas Courteau
"""

import numpy as np

from pyro.dynamic import cartpole
from pyro.control import nonlinear
from pyro.dynamic import mechanical

###############################################################################
# Computed Torque
###############################################################################

class ComputedTorqueControllerUnderactuated(nonlinear.ComputedTorqueController):
    """
    Inverse dynamic controller for mechanical system

    """

    ############################
    def __init__(self, model=mechanical.MechanicalSystem(), traj=None):
        """

        ---------------------------------------
        r  : reference signal vector  k x 1
        y  : sensor signal vector     p x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        ---------------------------------------
        u = c( y , r , t )

        """

        nonlinear.ComputedTorqueController.__init__(self, model, traj)

        self.c = self.c_flat

    ############################
    def compute_ddq_r(self, ddq_d, dq_d, q_d, dq, q):
        """

        Given desired trajectory and actual state, compute ddq_r

        """

        q_e = q - q_d
        dq_e = dq - dq_d

        ddq_r = ddq_d - 2 * self.zeta * self.w0 * dq_e - self.w0 ** 2 * q_e

        return ddq_r

    #############################
    def c_flat(self, y, r, t=0):
        """
        Feedback static computation u = c(y,r,t)

        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1

        OUPUTS
        u  : control inputs vector    m x 1

        """

        x = y
        q_d = r

        u = self.flat_ctl(x, q_d, t)

        return u

    ############################
    def flat_ctl(self, x, q_d, t=0):
        """

        Given desired fixed goal state and actual state, compute torques

        """

        u = np.zeros(self.m)

        # State feedback
        q = x[0:     self.model.dof]
        dq = x[self.model.dof: 2 * self.model.dof]

        u[0] = 50 * q[1] + 10 * dq[1] + 1 * (q[0] - q_d[0]) + 2 * dq[0]

        umax = 1  # [N]

        if u[0] > umax:
            u[0] = umax

        if u[0] < -umax:
            u[0] = -umax

        return u


###############################################################################

'''
#################################################################
##################          Main                         ########
#################################################################
'''

sys = cartpole.SelfBalanceRobot2D()

ctl = ComputedTorqueControllerUnderactuated(sys)

ctl.w0 = 1.0
ctl.zeta = 0.7

# goal
ctl.rbar = np.array([0., 0.])

# New cl-dynamic
cl_sys = ctl + sys

# Initiale state
# x0[0] : Position of the cart
# x0[1] : Angle of the rigid link
# x0[2] : Speed of the cart
# x0[3] : Angular speed of the rigid link
x0 = np.array([0, np.pi / 2 - 1.469, 0., 0])

# sys.plot_trajectory(x0, 2.5)  # 5, 50001, 'euler' )
# sys.animate_simulation(0.5, save=True)
cl_sys.plot_phase_plane_trajectory(x0)
cl_sys.sim.plot('xu')
cl_sys.animate_simulation(0.5, save=True)
