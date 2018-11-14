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
        
        # Parameters
        n    = ContinuousDynamicSystem.n
        m    = ContinuousDynamicSystem.m
        p    = ContinuousDynamicSystem.p
        
        raise NotImplementedError
        
    #############################
    def h(self, x , y = 0 , t = 0):
        """ Final cost function """
        
        raise NotImplementedError
    
    
    #############################
    def g(self, x , u , y = 0 , t = 0 ):
        """ step cost function """
        
        raise NotImplementedError
        

#############################################################################
     
class QuadraticCostFunction:
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
        
        # Parameters
        n = ContinuousDynamicSystem.n
        m = ContinuousDynamicSystem.m
        p = ContinuousDynamicSystem.p
        
        # Quadratic cost weights
        self.Q = np.diag( np.ones( n ) )
        self.R = np.diag( np.ones( m ) )
        self.V = np.diag( np.zeros( p ) )
        
    #############################
    def h(self, x , y , t = 0):
        """ Final cost function with zero value """
        
        return 0
    
    
    #############################
    def g(self, x , u , y , t = 0 ):
        """ Quadratic additive cost """
        
        dJ = ( np.dot( x.T , np.dot(  self.Q , x ) ) +
               np.dot( u.T , np.dot(  self.R , u ) ) +
               np.dot( y.T , np.dot(  self.V , y ) ) )
        
        return dJ
    

##############################################################################

class TimeCostFunction:
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
        
        pass
        
    #############################
    def h(self, x , y , t = 0):
        """ Final cost function with zero value """
        
        return 0
    
    
    #############################
    def g(self, x , u , y , t = 0 ):
        """ Unity """
        
        return 1

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass