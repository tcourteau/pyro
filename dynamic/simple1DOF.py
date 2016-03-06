# -*- coding: utf-8 -*-
"""
Created on Fri Aug 07 14:56:36 2015

@author: agirard
"""

import numpy as np
from AlexRobotics.dynamic import DynamicSystem as RDDS

'''
#################################################################
##################          MassSpringDamper             ########
#################################################################
'''

class MassSpringDamper( RDDS.DynamicSystem ) :
    """ Mass-spring-damper """
    
    ############################
    def __init__(self):
        
        n = 2   # number of states
        m = 1   # number of inputs
        
        RDDS.DynamicSystem.__init__(self, n , m )
    
    #############################
    def setparams(self, m = 1 , b = 1 , k = 1):
        """ """
        self.m = m
        self.b = b
        self.k = k
    
    #############################
    def fc(self, x = np.zeros(2) , u = np.zeros(1) , t = 0 ):
        """ 
        Continuous time function evaluation
        
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
        
        dx[0] = x[1]
        dx[1] = ( u[0] - self.b * x[1] - self.k * x[0] ) / self.m
        
        return dx


'''
#################################################################
##################          Pendumlum                    ########
#################################################################
'''

class Pendulum( RDDS.DynamicSystem ) :
    """ Pendulum """
    
        ############################
    def __init__(self):
        
        n = 2   # number of states
        m = 1   # number of inputs
        
        RDDS.DynamicSystem.__init__(self, n , m )
        
        self.state_label = ['Position [rad]','Speed [rad/sec]']
        self.input_label = ['Torque [Nm]']
    
    #############################
    def setparams(self, m = 1 , g = 9.8 , l = 1 ):
        """ """
        self.m = m
        self.l = l
        self.g = g
    
    #############################
    def fc(self, x = np.zeros(2) , u = np.zeros(1) , t = 0 ):
        """ 
        Continuous time function evaluation
        
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
        
        dx[0] = x[1]
        dx[1] = ( u[0] - self.m * self.g * self.l * np.sin( x[0] ) ) / ( self.m * self.l**2 )
        
        return dx
        
        

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    MSD = MassSpringDamper()
    P   = Pendulum()