# -*- coding: utf-8 -*-
"""
Created on Wed Jul 12 10:08:01 2017

@author: alxgr
"""

import numpy as np
import matplotlib.pyplot as plt


class PureQuadratic:
    """ Quadratic cost function class """
    
    ############################
    def __init__(self, n , m ):
        
        # Parameters
        self.n    = n
        self.m    = m
        self.INF  = 5000      #  default large cost
        
        # Quadratic cost
        self.Q     = np.diag( np.ones( n ) )
        self.R     = np.diag( np.ones( m ) )
        
        # Cost functions
        self.g      = self.g_quadratic
        self.h      = self.h_zero       # no final cost
        
        
    #############################
    def h(self,  x ):
        """ Final cost function """
        
        return 0
    
    
    #############################
    def g(self, x , u ):
        """ step cost function """
        
        return 1
        
        
    #############################
    def h_zero(self,  x ):
        """ Final cost function with zero value """
        
        return 0

        
    #############################
    def g_quadratic(self, x , u ):
        """ Quadratic additive cost """
        
         # On target not doing anything (don't count time at this point)
        cost = np.dot( x.T , np.dot(  self.Q , x ) ) + np.dot( u.T , np.dot(  self.R , u ) )
                
        return cost
        

        
    
    
##############################
#    def h_target(self,  x ):
#        """ Final cost function """
#        # Minimum time problem h = 0 , g = 1
#        
#        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ):
#            # On target = OK            
#            cost = 0 
#        else:
#            # Off target = bad
#            cost = self.INF
#        
#        return cost
#        
#    
#    #############################
#    def g(self, x , u ):
#        """ step cost function """
#        
#        return 1
#    
#    #############################
#    def g_time(self, x , u ):
#        """ Minimum time cost """
#
#        # On target not doing anything (don't count time at this point)
#        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ) and ( abs(u[0]) <= 0.1 ):
#            cost = 0
#        
#        # Add time for the move
#        else:
#            cost = self.dt # minimum time
#                
#        return cost
#
#        
#    #############################
#    def g_quadratic(self, x , u ):
#        """ Quadratic additive cost """
#         # On target not doing anything (don't count time at this point)
#        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ) and ( abs(u[0]) <= 0.1 ):
#            cost = 0
#        
#        # Add time for the move
#        else:
#            cost = ( self.w_quad[0] * x[0] ** 2 + self.w_quad[1] * x[1] ** 2 + self.w_quad[2] * u[0] ** 2 ) * self.dt
#                
#        return cost
#        
#        
#    #############################
#    def g_energy(self, x , u ):
#        """ Electric energy lost """
#
#        cost = ( self.w_quad[2] * u[0] ** 2 ) * self.dt # Energy
#                
#        return cost
        

        