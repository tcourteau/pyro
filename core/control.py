# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 08:40:31 2018

@author: alxgr
"""

import numpy as np

from AlexRobotics.core import system

###########################################################################################
# Mother Controller class
###########################################################################################

class StaticController:
    """ 
    Mother class for memoryless controllers
    ---------------------------------------
    r  : reference signal vector  k x 1
    y  : sensor signal vector     p x 1
    u  : control inputs vector    m x 1
    t  : time                     1 x 1
    ---------------------------------------
    u = c( y , r , t )
    
    """
    
    ###########################################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################################
    
    
    ############################
    def __init__(self):
        """ """
        # System parameters to be implemented
        
        # Dimensions
        self.k = 1   
        self.m = 1   
        self.p = 1
        
        # Label
        self.name = 'Controller'
        
        # Reference signal info
        self.ref_label = []
        self.ref_units = []
        self.r_ub      = np.zeros(self.k) + 10 # upper bounds
        self.r_lb      = np.zeros(self.k) - 10 # lower bounds
        
        # default constant reference
        self.rbar = np.zeros(self.k)
        
        raise NotImplementedError
        
    
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
        
        u = np.zeros(self.m) # State derivative vector
        
        raise NotImplementedError
        
        return u
    
    
    #########################################################################
    # No need to overwrite the following functions for child classes
    #########################################################################
    
    #############################
    def cbar( self , y , t = 0 ):
        """ 
        Feedback static computation u = c( y, r = rbar, t) for
        default reference
        
        INPUTS
        y  : sensor signal vector     p x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        u = self.c( y , self.rbar , t )
        
        return u
    
    

###########################################################################################
# Mother "Static controller + dynamic system" class
###########################################################################################

class ClosedLoopSystem( system.ContinuousDynamicSystem ):
    """ 
    Dynamic system connected with a static controller
    ---------------------------------------------
    NOTE: Ignore any feedthough to avoid creating algebraic loop
    new equations assume y = h(x,u,t) -- > y = h(x,t)

    """
    ############################
    def __init__(self, ContinuousDynamicSystem , StaticController):
        """ """
        
        self.sys = ContinuousDynamicSystem
        self.ctl = StaticController
        
        # Check dimentions match
        # TODO
        
        # Dimensions
        self.n = self.sys.n
        self.m = self.ctl.k 
        self.p = self.sys.p
        
        # Labels
        self.name = 'Closed-Loop ' + self.sys.name + ' with ' + self.sys.name
        self.state_label  = self.sys.state_label
        self.input_label  = self.ctl.ref_label
        self.output_label = self.sys.output_label
        
        # Units
        self.state_units = self.sys.state_units
        self.input_units = self.ctl.ref_units
        self.output_units = self.sys.output_units
        
        # Define the domain
        self.x_ub = self.sys.x_ub
        self.x_lb = self.sys.x_lb
        self.u_ub = self.ctl.r_ub
        self.u_lb = self.ctl.r_lb
        
        # Default State and inputs        
        self.xbar = self.sys.xbar
        self.ubar = self.ctl.rbar
        
    
    #############################
    def f( self , x , u , t ):
        """ 
        Continuous time foward dynamics evaluation dx = f(x,u,t)
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vector  n x 1
        
        """
        
        dx = np.zeros(self.n) # State derivative vector
        
        r = u # input is ref
        u = self.sys.ubar  # only for h
        
        dx = self.sys.f(  x  ,  self.ctl.c( self.sys.h(x,u,t) ,  r  ,  t  ),  t )
        
        return dx
    

    #############################
    def h( self , x , u , t ):
        """ 
        Output fonction y = h(x,u,t)
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUTPUTS
        y  : output derivative vector o x 1
        
        """
        
        #y = np.zeros(self.p) # Output vector
        
        y = self.sys.h( x , self.sys.ubar , t )
        
        return y
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass

    
