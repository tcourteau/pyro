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
    def ctl( self , x , t = 0 ):
        """ 
        u = f( x , t ) 
        x = [ position ; speed ]
        
        """
        u = np.zeros(1)
        
        u[0] =  -1 * ( self.kp * x[0] + self.kd * x[1] )
        
        return u
        
        
        
class PID:
    """ Feedback law  """
    ############################
    def __init__( self , kp = 1 , kd = 0 , ki = 0 , dt = 0.001 , setpoint = 0 ):
        """ """
        self.kp       = kp
        self.kd       = kd
        self.ki       = ki
        self.dt       = dt
        self.setpoint = setpoint
        
        # Init
        self.e_integral = 0
        
    ############################
    def reset( self ):
        """ Reinitialize integral action """
        
        self.e_integral = 0
        
        
    ############################
    def ctl( self , x , t = 0 ):
        """ 
        u = f( x , t ) 
        x = [ position ; speed ] --> state must be defined this way
        
        """
        u = np.zeros(1)
        
        e  = x[0] - self.setpoint
        de = x[1]                  # assuming stationnary setpoint
        ei = self.e_integral
        
        u[0] =  -1 * ( self.kp * e + self.kd * de + self.ki * ei )
        
        # Internal controller state dynamic
        self.e_integral = self.e_integral + e * self.dt
        
        return u
        
        
        
class PD_nDOF:
    """ Feedback law  """
    ############################
    def __init__( self , kp , kd , q_d = np.array([None]) , dq_d = np.array([None]) ):
        """ 
        Independant joint PD controller for a manipulator
        """
        
        self.dof = kp.size          # NUMBER OF dof
        self.n   = self.dof * 2     # number of states
        self.kp  = kp
        self.kd  = kd
        
        # Set to zero vector of good length if setpoint is undefined
        if q_d[0] == None:
            self.q_d = np.zeros( self.dof )
        else:
            self.q_d = q_d
            
        if dq_d[0] == None:
            self.dq_d = np.zeros( self.dof )
        else:
            self.dq_d = dq_d
    
        
    ############################
    def ctl( self , x , t = 0 ):
        """ 
        u = f( x , t ) 
        x = [ position ; speed ]  state must be defined this way
        
        """
        
        q  = x[  0        : self.dof ]
        dq = x[  self.dof : self.n   ]
        
        u  =   self.kp * ( self.q_d - q ) + self.kd *  ( self.dq_d - dq ) 
        
        return u