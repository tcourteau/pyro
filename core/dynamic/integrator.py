# -*- coding: utf-8 -*-
"""
Created on Thu Oct 18 20:54:31 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.core import system


#############################################################################

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
        self.name = 'Double Integrator'
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
    
##############################################################################
        
class SimpleIntegrator( system.ContinuousDynamicSystem ):
    """ 
    SimpleIntegrator Example for a ContinuousDynamicSystem
    -------------------------------------------------------
    mass driven by a speed input: dx [m/sec] = f(x,u,t) = u [m/sec]
    
    """
    
    ############################
    def __init__(self):
        """ """
    
        # Dimensions
        self.n = 1   
        self.m = 1   
        self.p = 1
        
        # Labels
        self.name = 'Simple Integrator'
        self.state_label = ['Position']
        self.input_label = ['Speed']
        self.output_label = ['Position']
        
        # Units
        self.state_units = ['[m]']
        self.input_units = ['[m/sec]']
        self.output_units = ['[m]']
        
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
        
        # Example simpleintergrator
        # x[0]: position u[0]: speed
        
        dx[0] = u[0]
        
        return dx
    
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    from AlexRobotics.core import analysis
    
    # Double integrator
    di = DoubleIntegrator()
    
    # Phase plane behavior test
    di.ubar = np.array([1])   
    di.plot_phase_plane()
    
    # Simulation
    x0 = np.array([0,0])
    di.plot_trajectory( x0 )
    
    # Cost computing
    di.sim.compute_cost()
    di.sim.plot('xuj')
    
    # Time cost
    di.sim.cf = analysis.TimeCostFunction( di )
    di.sim.compute_cost()
    di.sim.plot('j')
    
    # Phase plane trajectory
    di.plot_phase_plane_trajectory( x0 )
    
    # Simple integrator
    si = SimpleIntegrator()
    
    # Phase plane
    si.ubar = np.array([1])
    si.plot_phase_plane(0,0) # only one state for two axis!
    
    # Simulation
    si.plot_trajectory( np.array([2]) )
    si.sim.compute_cost()
    si.sim.plot('xuj')
    si.sim.phase_plane_trajectory(0,0)
    