# -*- coding: utf-8 -*-
"""
Created on Fri Aug 07 11:51:55 2015

@author: agirard
"""

import numpy as np

    
##########################################################################
# Cost functions
##########################################################################       

class CostFunction:
    """ 
    Mother class for cost functions of continuous dynamical systems
    ----------------------------------------------
    n : number of states
    m : number of control inputs
    p : number of outputs
    ---------------------------------------
    J = int( g(x,u,y,t) * dt ) + h( x(T) , y(T) , T )
    
    """
    ###########################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################
    
    ############################
    def __init__(self, ContinuousDynamicSystem ):
        
        self.sys = ContinuousDynamicSystem
        
        self.INF = 1E3
        self.EPS = 1E-3
        
        
    #############################
    def h(self, x , t = 0):
        """ Final cost function """
        
        raise NotImplementedError
    
    
    #############################
    def g(self, x , u , t = 0 ):
        """ step cost function """
        
        raise NotImplementedError
        

#############################################################################
     
class QuadraticCostFunction( CostFunction ):
    """ 
    Quadratic cost functions of continuous dynamical systems
    ----------------------------------------------
    n : number of states
    m : number of control inputs
    p : number of outputs
    ---------------------------------------
    J = int( g(x,u,y,t) * dt ) + h( x(T) , y(T) , T )
    
    g = xQx + uRu + yVy
    h = 0
    
    """
    
    ############################
    def __init__(self, ContinuousDynamicSystem ):
        
        CostFunction.__init__( self , ContinuousDynamicSystem )
        
        self.xbar = np.zeros( self.sys.n )
        self.ubar = np.zeros( self.sys.m )
        self.ybar = np.zeros( self.sys.p )
        
        # Quadratic cost weights
        self.Q = np.diag( np.ones( self.sys.n ) )
        self.R = np.diag( np.ones( self.sys.m ) )
        self.V = np.diag( np.zeros( self.sys.p ) )
        
        self.ontarget_check = True
        
    #############################
    def h(self, x , t = 0):
        """ Final cost function with zero value """
        
        return 0
    
    
    #############################
    def g(self, x , u , t = 0 ):
        """ Quadratic additive cost """
        
        y = self.sys.h( x , u , t )
        
        dx = x - self.xbar
        du = u - self.ubar
        dy = y - self.ybar
        
        dJ = ( np.dot( dx.T , np.dot(  self.Q , dx ) ) +
               np.dot( du.T , np.dot(  self.R , du ) ) +
               np.dot( dy.T , np.dot(  self.V , dy ) ) )
        
        # set cost to zero if on target
        if self.ontarget_check:
            if ( np.linalg.norm( dx ) < self.EPS ):
                dJ = 0
        
        return dJ
    

##############################################################################

class TimeCostFunction( CostFunction ):
    """ 
    Mother class for cost functions of continuous dynamical systems
    ----------------------------------------------
    n : number of states
    m : number of control inputs
    p : number of outputs
    ---------------------------------------
    J = int( g(x,u,y,t) * dt ) + h( x(T) , y(T) , T ) = T
    
    g = 1
    h = 0
    
    """
    
    ############################
    def __init__(self, ContinuousDynamicSystem ):
        
        CostFunction.__init__( self , ContinuousDynamicSystem )
        
        self.xbar = np.zeros( self.sys.n )
        
        self.ontarget_check = True
        
    #############################
    def h(self, x , t = 0):
        """ Final cost function with zero value """
        
        return 0
    
    
    #############################
    def g(self, x , u , t = 0 ):
        """ Unity """
        
        dJ = 1
        
        if self.ontarget_check:
            dx = x - self.xbar
            if ( np.linalg.norm( dx ) < self.EPS ):
                dJ = 0
                
        return dJ

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass