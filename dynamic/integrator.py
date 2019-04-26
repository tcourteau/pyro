# -*- coding: utf-8 -*-
"""
Created on Thu Oct 18 20:54:31 2018

@author: Alexandre
"""

import numpy as np

from PyRobotics.dynamic import system


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
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
        
        # Labels
        self.name = 'Simple Integrator'
        self.state_label = ['Position']
        self.input_label = ['Speed']
        self.output_label = ['Position']
        
        # Units
        self.state_units = ['[m]']
        self.input_units = ['[m/sec]']
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
        
        # Example simpleintergrator
        # x[0]: position u[0]: speed
        
        dx[0] = u[0]
        
        return dx


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
        self.p = 1
        
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
    
    
    #############################
    def h( self , x , u , t ):
        """ 
        Output fonction y = h(x,u,t)
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUTPUTS
        y  : output derivative vector p x 1
        
        """
        
        y = np.zeros(self.p) # Output vector
        
        y[0] = x[0] # output is first state = position
        
        return y
    
    
    
##############################################################################
        
    
class TripleIntegrator( system.ContinuousDynamicSystem ):
    """ 
    DoubleIntegrator Example for a ContinuousDynamicSystem
    
    """
    ############################
    def __init__(self):
        """ """
    
        # Dimensions
        self.n = 3   
        self.m = 1   
        self.p = 1
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
        
        # Labels
        self.name = 'Triple Integrator'
        self.state_label = ['Position','Speed', 'Force']
        self.input_label = ['Gradient of Force']
        self.output_label = ['Position']
        
        # Units
        self.state_units = ['[m]','[m/sec]','[N]']
        self.input_units = ['[N/sec]']
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
        
        # x[0]: position 
        # x[1]: speed 
        # x[2]: force
        
        dx[0] = x[1]  # 
        dx[1] = x[2]  # 
        dx[2] = u[0]  # 
        
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
        
        y = np.zeros(self.p) # Output vector
        
        y[0] = x[0] # output is first state = position
        
        return y
    
    
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    from PyRobotics.analysis import costfunction
    
    ###################################
    # Simple integrator
    ###################################
    
    si = SimpleIntegrator()
    
    # Phase plane
    si.ubar = np.array([1])
    si.plot_phase_plane(0,0) # only one state for two axis!
    
    # Simulation
    si.plot_trajectory( np.array([2]) )
    si.sim.compute_cost()
    si.sim.plot('xuj')
    si.sim.phase_plane_trajectory(0,0)
    
    ###################################
    # Double integrator
    ###################################
    
    di = DoubleIntegrator()
    
    # Phase plane behavior test
    di.ubar = np.array([1])   
    di.plot_phase_plane()
    
    # Simulation
    x0 = np.array([0,0])
    di.plot_trajectory( x0 )
    di.sim.plot('y')
    
    # Cost computing
    di.sim.compute_cost()
    di.sim.plot('xuj')
    
    # Time cost
    di.sim.cf = costfunction.TimeCostFunction( di )
    di.sim.compute_cost()
    di.sim.plot('j')
    
    # Phase plane trajectory
    di.plot_phase_plane_trajectory( x0 )

    ###################################
    # Simple integrator
    ###################################
    
    ti = TripleIntegrator()
    
    # Phase plane
    ti.ubar = np.array([1])
    
    # Simulation
    ti.plot_trajectory( np.array([2,0,0]) )
    ti.sim.compute_cost()
    ti.sim.plot('xu')
    ti.sim.phase_plane_trajectory(0,1)
    