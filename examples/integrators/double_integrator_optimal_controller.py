# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 10:17:36 2018

@author: Alexandre
"""

###############################################################################
import numpy as np

###############################################################################
from pyro.dynamic  import system
from pyro.analysis import costfunction
from pyro.control  import controller
###############################################################################

#############################################################################

class DoubleIntegrator2( system.ContinuousDynamicSystem ):
    """ 
    DoubleIntegrator Example for a ContinuousDynamicSystem
    
    """
    
    ############################
    def __init__(self):
        """ """
        
        # Dimensions
        self.n = 2   
        self.m = 1   
        self.p = 2
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
    
        # Labels
        self.name = 'Double Integrator'
        self.state_label = ['Position','Speed']
        self.input_label = ['Force']
        self.output_label = ['Position']
        
        # Units
        self.state_units = ['[m]','[m/sec]']
        self.input_units = ['[N]']
        self.output_units = ['[m]']
        
    
    #############################
    def f(self, x = np.zeros(2) , u = np.zeros(1) , t = 0 ):
        """ 
        Continuous time foward dynamics evaluation
        
        dx = f(x,u,t)
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        dx = np.zeros(self.n) # State derivative vector
        
        # Example double intergrator
        # x[0]: position x[1]: speed
        
        dx[0] = x[1]  # 
        dx[1] = u[0]  # 
        
        return dx
    


class OptmimalDoubleIntegratorController( controller.StaticController ) :
    
    ############################
    def __init__( self):
        """ """
        
        # Dimensions
        self.dof = 1
        self.k   = 1 
        self.m   = 1
        self.p   = 2
        
        controller.StaticController.__init__(self, self.k, self.m, self.p)
        
        # Label
        self.name = 'Optmimal Double Integrator Controller'
        
        self.xd = 0

        self.rbar = np.zeros(1)
        
    
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
        
        u = np.zeros(1) 
        
        x  = y[0]
        dx = y[1]
        
        xe = x - self.xd
        
        if (dx > np.sign(-xe) * np.sqrt(np.abs(2 * ( xe )))) :
            u[0] = -1
        else:
            u[0] = 1

        
        return u

    
###################################
# DOuble integrator
###################################

sys = DoubleIntegrator2()


ctl = OptmimalDoubleIntegratorController()

ctl.xd = 5

# New cl-dynamic
cl_sys = ctl + sys

x0 = np.array([-2,0])
cl_sys.plot_trajectory( x0 , 10 , 10001, 'euler')
cl_sys.sim.plot('xu')
cl_sys.sim.phase_plane_trajectory(0,1)