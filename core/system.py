# -*- coding: utf-8 -*-
"""
Created on Fri Aug 07 11:51:55 2015

@author: agirard
"""

import numpy as np

from AlexRobotics.core import analysis

       
'''
################################################################################
'''


class ContinuousDynamicSystem:
    """ 
    Mother class for continuous dynamical systems
    ----------------------------------------------
    n : number of states
    m : number of control inputs
    p : number of outputs
    ---------------------------------------
    dx = f( x , u , t )
    y  = h( x , u , t )
    
    """
    ###########################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################
    
    ############################
    def __init__(self, n = 1, m = 1, p = 1):
        """ 
        The __init__ method of the Mother class can be used to fill-in default
        labels, units, bounds, and base values.
        """

        # Dimensions
        self.n = n   
        self.m = m   
        self.p = p
        
        # Labels
        self.name = 'ContinuousDynamicSystem'
        self.state_label  = []
        self.input_label  = []
        self.output_label = []
        
        # Units
        self.state_units  = []
        self.input_units  = []
        self.output_units = []
        
        # Default Label and units
        for i in range(n):
            self.state_label.append('State '+str(i))
            self.state_units.append('')
        for i in range(m):
            self.input_label.append('Input '+str(i))
            self.input_units.append('')
        for i in range(p):
            self.output_label.append('Output '+str(i))
            self.output_units.append('')
        
        # Default state and input domain
        self.x_ub = np.zeros(self.n) +10 # States Upper Bounds
        self.x_lb = np.zeros(self.n) -10 # States Lower Bounds
        self.u_ub = np.zeros(self.m) +1  # Control Upper Bounds
        self.u_lb = np.zeros(self.m) -1  # Control Lower Bounds
        
        # Default state and inputs values    
        self.xbar = np.zeros(self.n)
        self.ubar = np.zeros(self.m)
        
    
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
        
        ################################################
        # Place holder: put the equations of motion here
        ################################################
        
        raise NotImplementedError
        
        return dx
    
    
    ###########################################################################################
    # The following functions can be overloaded when necessary by child classes
    ###########################################################################################
    
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
        
        y = x      # default output is all states
        
        return y
    
        
    ###########################################################################################
    # Basic domain checks, ovewrite if something more complex is needed, ex: obstacles, etc
    ###########################################################################################
        
    #############################
    def isavalidstate(self , x ):
        """ check if x is in the state domain """
        ans = False
        for i in range(self.n):
            ans = ans or ( x[i] < self.x_lb[i] )
            ans = ans or ( x[i] > self.x_ub[i] )
            
        return not(ans)
        
    #############################
    def isavalidinput(self , x , u):
        """ check if u is in the control inputs domain given x """
        ans = False
        for i in range(self.m):
            ans = ans or ( u[i] < self.u_lb[i] )
            ans = ans or ( u[i] > self.u_ub[i] )
            
        return not(ans)
        
    
    
    #########################################################################
    # No need to overwrite the following functions for custom dynamic systems
    #########################################################################
    
    #############################
    def fbar( self , x , t ):
        """ 
        Continuous time foward dynamics evaluation dx = f( x , u = ubar , t )
        for default constant control input (open-loop)
        
        INPUTS
        x  : state vector             n x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vector  n x 1
        
        """
        
        dx = self.f( x , self.ubar , t )
        
        return dx
    
        
    #############################
    def x_next( self , x , u , dt = 0.1 ):
        """ 
        Discrete time foward dynamics evaluation 
        -------------------------------------
        - using Euler integration
        
        """
        
        x_next = np.zeros(self.n) # k+1 State vector
        
        x_next = self.f(x,u) * dt + x
        
        return x_next
        
        
    #############################
    def plot_phase_plane(self , x_axis = 0 , y_axis = 1 ):
        """ 
        Plot Phase Plane vector field of the system
        ------------------------------------------------
        x_axis : index of state on x axis
        y_axis : index of state on y axis
        
        """

        self.pp = analysis.PhasePlot( self , x_axis , y_axis )
        
        self.pp.plot()
        
        
    #############################
    def plot_trajectory(self , x0 , tf = 10 ):
        """ 
        Simulation of time evolution of the system
        ------------------------------------------------
        x0 : initial time
        tf : final time
        
        """

        self.sim = analysis.Simulation( self , tf )
        
        self.sim.x0 = x0
        self.sim.compute()
        
        self.sim.plot()
        
        
    #############################
    def plot_phase_plane_trajectory(self , x0 , tf = 10 , x_axis = 0 , y_axis = 1):
        """ 
        Simulates the system and plot the trajectory in the Phase Plane 
        ---------------------------------------------------------------
        x0 : initial time
        tf : final time
        x_axis : index of state on x axis
        y_axis : index of state on y axis
        
        """
        
        self.sim = analysis.Simulation( self , tf )
        
        self.sim.x0 = x0
        self.sim.compute()
        self.sim.phase_plane_trajectory( x_axis , y_axis )


'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass