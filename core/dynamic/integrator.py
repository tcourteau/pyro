# -*- coding: utf-8 -*-
"""
Created on Thu Oct 18 20:54:31 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.core import system


class DoubleIntegrator( system.ContinuousDynamicSystem ):
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
        
        # Labels
        self.state_label = ['Position','Speed']
        self.input_label = ['Force']
        self.output_label = ['Position','Speed']
        
        # Units
        self.state_units = ['[m]','[m/sec]']
        self.input_units = ['[N]']
        self.output_units = ['[m]','[m/sec]']
        
        # Define the domain
        self.x_ub = np.zeros(self.n) + 10 # States Upper Bounds
        self.x_lb = np.zeros(self.n) - 10 # States Lower Bounds
        self.u_ub = np.zeros(self.m) + 10 # Control Upper Bounds
        self.u_lb = np.zeros(self.m) - 10 # Control Lower Bounds
        
        # Default State and inputs        
        self.xbar = np.zeros(self.n)
        self.ubar = np.zeros(self.m)
        
    
    #############################
    def f(self, x = np.zeros(2) , u = np.zeros(1) , t = 0 ):
        """ 
        Continuous time foward dynamics evaluation
        
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
        dx[1] = u[0]
        
        return dx
    
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    # Default is double integrator
    di = DoubleIntegrator()
    
    # Phase plane behavior with open-loop u=3, strating at [-5,-5] for 7 sec
    di.ubar = np.array([0])
    x0      = np.array([0,0])
    tf      = 7
    
    
    di.phase_plane()
    #DS.phase_plane_trajectory( x0 , tf )