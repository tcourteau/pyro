# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 14:59:30 2016

@author: alex
"""

import numpy as np

'''
################################################################################
'''

    
class PD:
    """ Feedback law  """
    ############################
    def __init__( self , kp , kd ):
        """ """
        self.kp = kp
        self.kd = kd
        
    ############################
    def u( self , x , t = 0 ):
        """ 
        u = f( x , t ) 
        x = [ position ; speed ]
        
        """
        u = np.zeros(1)
        
        u[0] =  -1 * ( self.kp * x[0] + self.kd * x[1] )
        
        return u
        
        
        
class PD_nDOF:
    """ Feedback law  """
    ############################
    def __init__( self , kp , kd , q_d = 0 , dq_d = 0):
        """ 
        Independant joint PD controller for a manipulator
        """
        
        self.n  = kp.size
        self.kp = kp
        self.kd = kd
        
        if q_d == 0:
            self.q_d = np.zeros(self.n)
        else:
            self.q_d = q_d
            
        if dq_d == 0:
            self.dq_d = np.zeros(self.n)
        else:
            self.dq_d = dq_d
    
        
    ############################
    def u( self , x , t = 0 ):
        """ 
        u = f( x , t ) 
        x = [ position ; speed ]
        
        """
        
        q  = x[0:2]
        dq = x[2:4]
        
        u  =   self.kp * ( self.q_d - q ) + self.kd *  ( self.dq_d - dq ) 
        
        return u