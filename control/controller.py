# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 08:40:31 2018

@author: alxgr
"""

import numpy as np

from AlexRobotics.dynamic import system
from AlexRobotics.analysis import phaseanalysis
from AlexRobotics.analysis import simulation
from AlexRobotics.analysis import graphical

###############################################################################
# Mother Controller class
###############################################################################

class StaticController:
    """ 
    Mother class for memoryless controllers
    ---------------------------------------
    r  : reference signal vector  k x 1
    y  : sensor signal vector     p x 1
    u  : control inputs vector    m x 1
    t  : time                     1 x 1
    ---------------------------------------
    u = c( y , r , t )
    
    """
    
    ###########################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################
    
    
    ############################
    def __init__(self, k=1, m=1, p=1):
        """ """
        # System parameters to be implemented
        
        # Dimensions
        self.k = k   
        self.m = m   
        self.p = p
        
        # Label
        self.name = 'StaticController'
        
        # Reference signal info
        self.ref_label = []
        self.ref_units = []
        
        for i in range(k):
            self.ref_label.append('Ref. '+str(i))
            self.ref_units.append('')
        
        self.r_ub = np.zeros(self.k) + 10 # upper bounds
        self.r_lb = np.zeros(self.k) - 10 # lower bounds
        
        # default constant reference
        self.rbar = np.zeros(self.k)
        
    
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback static computation u = c(y,r,t)
        
        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        u = np.zeros(self.m) # State derivative vector
        
        raise NotImplementedError
        
        return u
    
    
    #########################################################################
    # No need to overwrite the following functions for child classes
    #########################################################################
    
    #############################
    def cbar( self , y , t = 0 ):
        """ 
        Feedback static computation u = c( y, r = rbar, t) for
        default reference
        
        INPUTS
        y  : sensor signal vector     p x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        u = self.c( y , self.rbar , t )
        
        return u
    
    #############################
    def __add__(self, sys):
        """ 
        closed_loop_system = controller + dynamic_system
        """
        
        cl_sys = ClosedLoopSystem( sys , self )
        
        return cl_sys
    
    
###############################################################################
# Mother "Static controller + dynamic system" class
###############################################################################

class ClosedLoopSystem( system.ContinuousDynamicSystem ):
    """ 
    Dynamic system connected with a static controller
    ---------------------------------------------
    NOTE: 
    Ignore any feedthough to avoid creating algebraic loop
    This is only valid if the output function h is not a fonction of u
    New equations assume y = h(x,u,t) -- > y = h(x,t)

    """
    ############################
    def __init__(self, ContinuousDynamicSystem , StaticController):
        """ """
        
        self.sys = ContinuousDynamicSystem
        self.ctl = StaticController
        
        ######################################################################
        # Check dimensions match
        if not (self.sys.m == self.ctl.m ):
            raise NameError('Dimension mismatch between controller and' + 
            ' dynamic system for the input signal u')
        elif not (self.sys.p == self.ctl.p ):
            raise NameError('Dimension mismatch between controller and' + 
            ' dynamic system for the output signal y')
        ######################################################################
        
        # Dimensions of global closed-loop dynamic system
        self.n = self.sys.n
        self.m = self.ctl.k 
        self.p = self.sys.p
        
        # Labels
        self.name = 'Closed-Loop ' + self.sys.name + ' with ' + self.ctl.name
        self.state_label  = self.sys.state_label
        self.input_label  = self.ctl.ref_label
        self.output_label = self.sys.output_label
        
        # Units
        self.state_units = self.sys.state_units
        self.input_units = self.ctl.ref_units
        self.output_units = self.sys.output_units
        
        # Define the domain
        self.x_ub = self.sys.x_ub
        self.x_lb = self.sys.x_lb
        self.u_ub = self.ctl.r_ub
        self.u_lb = self.ctl.r_lb
        
        # Default State and inputs        
        self.xbar = self.sys.xbar
        self.ubar = self.ctl.rbar
        
    
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
        
        r = u # input of closed-loop global sys is ref of the controller
        y = self.sys.h( x, self.sys.ubar, t)
        u = self.ctl.c( y, r, t)
        
        dx = self.sys.f( x, u, t)
        
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
        
        #y = np.zeros(self.p) # Output vector
        
        y = self.sys.h( x , self.sys.ubar , t )
        
        return y
    
    #############################
    def plot_phase_plane_closed_loop(self , x_axis = 0 , y_axis = 1 ):
        """ 
        Plot Phase Plane vector field of the system
        ------------------------------------------------
        
        x_axis : index of state on x axis
        y_axis : index of state on y axis
        
        """

        self.pp = phaseanalysis.PhasePlot( self , x_axis , y_axis )
        
        self.pp.compute_grid()
        self.pp.plot_init()
        
        # Closed-loop Behavior
        self.pp.color = 'r'
        self.pp.compute_vector_field()
        self.pp.plot_vector_field()
        
        # Open-Loop Behavior
        self.pp.f     = self.sys.f
        self.pp.ubar  = self.sys.ubar
        self.pp.color = 'b'
        self.pp.compute_vector_field()
        self.pp.plot_vector_field()
        
        self.pp.plot_finish()
        
    
    #############################
    def compute_trajectory(self , x0 , tf = 10 , n = 10001 , solver = 'ode'):
        """ 
        Simulation of time evolution of the system
        ------------------------------------------------
        x0 : initial time
        tf : final time
        
        """
        
        self.sim = simulation.CLosedLoopSimulation( self , tf , n , solver )
        self.sim.x0 = x0
        self.sim.compute()
        
        
    #############################
    def plot_trajectory(self , x0 , tf = 10 , n = 10001 , solver = 'ode'):
        """ 
        Simulation of time evolution of the system
        ------------------------------------------------
        x0 : initial time
        tf : final time
        
        """

        self.compute_trajectory( x0 , tf , n , solver )
        
        self.sim.plot('xu')
        
        
    #############################
    def plot_phase_plane_trajectory(self, x0, tf=10, x_axis=0, y_axis=1):
        """ 
        Simulates the system and plot the trajectory in the Phase Plane 
        ------------------------------------------------
        x0 : initial time
        tf : final time
        x_axis : index of state on x axis
        y_axis : index of state on y axis
        
        """
        
        self.sim = simulation.CLosedLoopSimulation( self , tf )
        
        self.sim.x0 = x0
        self.sim.compute()
        self.sim.phase_plane_trajectory_closed_loop( x_axis , y_axis )
        
    
    #############################
    def plot_phase_plane_trajectory_3d(self , x0, tf=10,
                                     x_axis=0, y_axis=1, z_axis=2):
        """ 
        Simulates the system and plot the trajectory in the Phase Plane 
        ---------------------------------------------------------------
        x0 : initial time
        tf : final time
        x_axis : index of state on x axis
        y_axis : index of state on y axis
        
        """
        
        self.sim = simulation.CLosedLoopSimulation( self , tf )
        
        self.sim.x0 = x0
        self.sim.compute()
        self.sim.phase_plane_trajectory_3d( x_axis , y_axis , z_axis )
        
    #############################################
    # Make graph function use the internal sys
    #############################################
        
    #############################################
    def show(self, q , x_axis = 0 , y_axis = 1 ):
        """ Plot figure of configuration q """
        
        system.ContinuousDynamicSystem.show( self.sys , q , 
                                            x_axis = 0 , y_axis = 1  )
        
    
    #############################################
    def show3(self, q ):
        """ Plot figure of configuration q """
        
        system.ContinuousDynamicSystem.show3(self.sys, q)
    
    #############################
    def plot_animation(self, x0 , tf = 10 , n = 10001 , solver = 'ode' ):
        """ Simulate and animate system """
        
        self.compute_trajectory( x0 , tf , n , solver )
        
        self.ani = graphical.Animator( self.sys )
        self.ani.sys.sim = self.sim
        self.ani.animate_simulation( 1.0 )
        
    ##############################
    def animate_simulation(self, time_factor_video =  1.0 , is_3d = False, 
                           save = False , file_name = 'RobotSim' ):
        """ 
        Show Animation of the simulation 
        ----------------------------------
        time_factor_video < 1 --> Slow motion video        
        
        """  
        
        self.ani = graphical.Animator( self.sys )
        self.ani.sys.sim = self.sim
        self.ani.animate_simulation( time_factor_video , is_3d, 
                                     save , file_name )

        
        
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass

    
