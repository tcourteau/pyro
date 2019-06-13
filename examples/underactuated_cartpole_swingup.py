# -*- coding: utf-8 -*-
"""
Created on Wed Jun 12 20:13:30 2019

@author: alxgr
"""

###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic import cartpole
from pyro.control import controller
###############################################################################

class CartPoleController( controller.StaticController ) :
    
    ############################
    def __init__( self , sys_model ):
        """ """
        
        # Dimensions
        self.dof = 2
        self.k   = 1 
        self.m   = 1
        self.p   = 4 
        
        controller.StaticController.__init__(self, self.k, self.m, self.p)
        
        # Label
        self.name = 'Cart Pole Controller'
        
        # Gains
        self.kd = 5
        self.kp = 3
        
        self.rbar = np.zeros(1)
        
        
        # Model
        self.sys = sys_model

    
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback static computation u = c(y,r,t)
        
        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        u = np.zeros(self.m) 
        
        # State feedback
        q  = y[ 0        :     self.dof   ]
        dq = y[ self.dof : 2 * self.dof   ]
        
        H = self.sys.H(q)
        C = self.sys.C(q,dq)
        d = self.sys.d(q,dq)
        g = self.sys.g(q)
        
        phi = np.dot(C,dq) + d + g
        
        H11 = H[0,0]
        H12 = H[0,1]
        H22 = H[1,1]
        
        A = H12 - H11 * H22 / H12
        B = phi[0] - H11 / H12 * phi[1]
        
        ddq_r = - self.kd * dq[1] - self.kp * q[1]
        
        T = 0.5 * H22 * dq[1]**2
        V = self.sys.m2 * self.sys.gravity * self.sys.l2 * np.cos( q[1] ) 
        E = T + V
        
        Ed = self.sys.m2 * self.sys.gravity * self.sys.l2
        
        Ee = E-Ed
    
        if ( np.abs(H12) > 0.01 ) & ( dq[1] > 0.1 ):
            
            mgl = self.sys.m2 * self.sys.gravity * self.sys.l2 * np.sin( q[1])
            
            ddq_r = (mgl - Ee / dq[1] ) / H22
            
            u[0] = A * ddq_r + B - 5 * dq[0]
                
 
        
        return u



###############################################################################

sys = cartpole.UnderActuatedRotatingCartPole()

sys.d1 = 0
sys.d2 = 0

ctl = CartPoleController( sys )

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x0 = np.array([0,-1.14,0,0])
cl_sys.plot_trajectory( x0 , 10 , 10001, 'euler')
cl_sys.sim.plot('xu')
cl_sys.animate_simulation(1.0,True)