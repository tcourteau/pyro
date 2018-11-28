# -*- coding: utf-8 -*-
"""
Created on Fri Aug 07 11:51:55 2015

@author: agirard
"""

import numpy as np

from AlexRobotics.analysis import simulation
from AlexRobotics.analysis import phaseanalysis
from AlexRobotics.analysis import graphical
       
'''
###############################################################################
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
    
    
    ###########################################################################
    # The following functions can be overloaded when necessary by child classes
    ###########################################################################
    
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
    
        
    ###########################################################################
    # Basic domain checks, ovewload if something more complex is needed
    ###########################################################################
        
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
    
    
    ###########################################################################
    # Place holder graphical output, ovewload with specific graph output
    ###########################################################################
        
    #############################
    def xut2q( self, x , u , t ):
        """ Compute configuration variables """
        
        # default is q = x
        
        return x
    
    ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 10
        
        domain  = [ (-l,l) , (-l,l) , (-l,l) ]#  
                
        return domain
    
    ###########################################################################
    def forward_kinematic_lines(self, q ):
        """ 
        Compute points p = [x;y;z] positions given config q 
        ----------------------------------------------------
        - points of interest for ploting
        
        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines
        
        """
        
        lines_pts = [] # list of array (n_pts x 3) for each lines
        
        ###########################
        # Your graphical code here
        ###########################
            
        # simple place holder
        for i in range(self.n):
            pts      = np.zeros(( 1 , 3 ))     # array of 1 pts for the line
            pts[0,0] = q[i]                    # x cord of point 0 = q
            lines_pts.append( pts )            # list of all arrays of pts
                
        return lines_pts
    
    ###########################################################################
    # No need to overwrite the following functions for custom dynamic systems
    ###########################################################################
    
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
    def x_next( self , x , u , t , dt = 0.1 , steps = 1 ):
        """ 
        Discrete time foward dynamics evaluation 
        -------------------------------------
        - using Euler integration
        
        """
        
        x_next = np.zeros(self.n) # k+1 State vector
        
        # Multiple integration steps
        for i in range(steps):
        
            x_next = self.f(x,u,t) * dt + x
            
            # Multiple steps
            x =  x_next
        
        return x_next
    
    
    ###########################################################################
    # Quick Analysis Shorcuts
    ###########################################################################
        
    #############################
    def plot_phase_plane(self , x_axis = 0 , y_axis = 1 ):
        """ 
        Plot Phase Plane vector field of the system
        ------------------------------------------------
        x_axis : index of state on x axis
        y_axis : index of state on y axis
        
        """

        self.pp = phaseanalysis.PhasePlot( self , x_axis , y_axis )
        
        self.pp.plot()
        
        
    #############################
    def compute_trajectory(self , x0 , tf = 10 , n = 10001 , solver = 'ode'):
        """ 
        Simulation of time evolution of the system
        ------------------------------------------------
        x0 : initial time
        tf : final time
        
        """
        
        self.sim = simulation.Simulation( self , tf , n , solver )
        self.sim.x0 = x0
        self.sim.compute()
        
    #############################
    def plot_trajectory(self , x0 , tf = 10 ):
        """ 
        Simulation of time evolution of the system
        ------------------------------------------------
        x0 : initial time
        tf : final time
        
        """

        self.compute_trajectory( x0 , tf )
        
        self.sim.plot()
        
        
    #############################
    def plot_phase_plane_trajectory(self , x0, tf=10, x_axis=0, y_axis=1):
        """ 
        Simulates the system and plot the trajectory in the Phase Plane 
        ---------------------------------------------------------------
        x0 : initial time
        tf : final time
        x_axis : index of state on x axis
        y_axis : index of state on y axis
        
        """
        
        self.sim = simulation.Simulation( self , tf )
        
        self.sim.x0 = x0
        self.sim.compute()
        self.sim.phase_plane_trajectory( x_axis , y_axis )
        
    
    #############################################
    def show(self, q , x_axis = 0 , y_axis = 1 ):
        """ Plot figure of configuration q """
        
        self.ani = graphical.Animator( self )
        self.ani.x_axis  = x_axis
        self.ani.y_axis  = y_axis
        
        self.ani.show( q )
        
    
    #############################################
    def show3(self, q ):
        """ Plot figure of configuration q """
        
        self.ani = graphical.Animator( self )
        
        self.ani.show3( q )
    
    #############################
    def animate(self, x0 , tf = 10 , n = 10001 , solver = 'ode' ):
        """ Simulate and animate system """
        
        self.compute_trajectory( x0 , tf , n , solver )
        
        self.ani = graphical.Animator( self )
        self.ani.animate_simulation( 1.0 )
        
    ##############################
    def animate_simulation(self,linestyle = 'o-', time_factor_video =  1.0 , is_3d = False, save = False , file_name = 'RobotSim' ):
        """ 
        Show Animation of the simulation 
        ----------------------------------
        time_factor_video < 1 --> Slow motion video        
        
        """  
        self.linestyle = linestyle
        self.ani = graphical.Animator( self ,self.linestyle)
        self.ani.animate_simulation( time_factor_video , is_3d, save , file_name )


'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass