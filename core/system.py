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
    
    """
    
    
    """
    ###########################################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################################
    """
    
    ############################
    def __init__(self):
        """ """
        
        # System parameters to be implemented
        
        # Dimensions
        self.n = 1   
        self.m = 1   
        self.p = 1
        
        # Labels
        self.name = 'ContinuousDynamicSystem'
        self.state_label = []
        self.input_label = []
        self.output_label = []
        
        # Units
        self.state_units = []
        self.input_units = []
        self.output_units = []
        
        # Define the domain
        self.x_ub = np.zeros(self.n) +10 # States Upper Bounds
        self.x_lb = np.zeros(self.n) +10 # States Lower Bounds
        self.u_ub = np.zeros(self.m) +1  # Control Upper Bounds
        self.u_lb = np.zeros(self.m) +1  # Control Lower Bounds
        
        # Default State and inputs        
        self.xbar = np.zeros(self.n)
        self.ubar = np.zeros(self.m)
        
        raise NotImplementedError
        
    
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
        
        raise NotImplementedError
        
        return dx
    
    
    """
    ###########################################################################################
    # The following functions can be overloaded when necessary by child classes
    ###########################################################################################
    """
    
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
        
    
    """
    #########################################################################
    # No need to overwrite the following functions for custom dynamic systems
    #########################################################################
    """
    
    #############################
    def fo( self , x , t ):
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
        ------------------------------------------------
        x0 : initial time
        tf : final time
        x_axis : index of state on x axis
        y_axis : index of state on y axis
        
        """
        
        self.sim = analysis.Simulation( self , tf )
        
        self.sim.x0 = x0
        self.sim.compute()
        self.sim.phase_plane_trajectory( x_axis , y_axis )
        
        

class DoubleIntegrator( ContinuousDynamicSystem ):
    """ 
    DoubleIntegrator Example for a ContinuousDynamicSystem
    
    """
        
    #############################
    def setparams(self):
        """ 
        Parameters initialization
        --------------------------------------
        Set model parameters here
        
        """
        
        pass
        
    
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
################################################################################
'''


    
class Simulation:
    """ Time simulation of a dynamic system  """
    ############################
    def __init__(self, DynamicSystem , tf = 10 , n = 10001 , solver = 'ode' ):
        
        self.DS = DynamicSystem
        self.t0 = 0
        self.tf = tf
        self.n  = int(n)
        self.dt = ( tf + 0.0 - self.t0 ) / ( n - 1 )
        
        self.x0 = np.zeros( self.DS.n )
        
        self.solver = solver
        
        # Ploting
        self.fontsize = 5
        
        # Cost computing
        self.J = 0
        self.Q = np.diag( np.ones( self.DS.n) )  # State cost per unit of time
        self.R = np.diag( np.ones( self.DS.m) )  # Input cost per unit of time
        self.H = np.diag( np.ones( self.DS.n) )  # Final State cost per unit of time
        
        self.domain_check     = False
        self.domain_fail_cost = 10
        
    ##############################
    def compute(self):
        """ Integrate trought time """
        
        self.t  = np.linspace( self.t0 , self.tf , self.n )
        self.dt = ( self.tf + 0.0 - self.t0 ) / ( self.n - 1 )
        
        self.J  = 0
        
        if self.solver == 'ode':
        
            self.x_sol_OL = odeint( self.DS.fc_OpenLoop   , self.x0 , self.t)
            self.x_sol_CL = odeint( self.DS.fc_ClosedLoop , self.x0 , self.t)
            
            # Compute control inputs
            self.u_sol_CL   = np.zeros(( self.n , self.DS.m ))  
            
            for i in range(self.n):
                
                # u & x
                u = self.DS.ctl( self.x_sol_CL[i,:] , self.t[i] )
                x = self.x_sol_CL[i,:]
                
                # integral cost
                xQx    = np.dot( x.T , np.dot( self.Q , x ) )
                uRu    = np.dot( u.T , np.dot( self.R , u ) )
                self.J = self.J + ( xQx + uRu ) * self.dt
                
                # domain check
                if self.domain_check:
                    if not( self.DS.isavalidstate( x ) ):
                        self.J = self.J + self.domain_fail_cost 
                    if not( self.DS.isavalidinput( x , u ) ):
                        self.J = self.J + self.domain_fail_cost 
                
                self.u_sol_CL[i,:] = u
                
                
        elif self.solver == 'euler':
            
            self.x_sol_OL = np.zeros((self.n,self.DS.n))
            self.x_sol_CL = np.zeros((self.n,self.DS.n))
            self.u_sol_CL = np.zeros((self.n,self.DS.m))
            
            # Initial State    
            self.x_sol_OL[0,:] = self.x0
            self.x_sol_CL[0,:] = self.x0
            
            for i in range(self.n):
                
                # u & x
                u = self.DS.ctl( self.x_sol_CL[i,:] , self.t[i] )
                x = self.x_sol_CL[i,:]
                
                # integral cost
                xQx    = np.dot( x.T , np.dot( self.Q , x ) )
                uRu    = np.dot( u.T , np.dot( self.R , u ) )
                self.J = self.J + ( xQx + uRu ) * self.dt
                
                self.u_sol_CL[i,:] = u.copy()
                
                if i+1<self.n:
                    self.x_sol_CL[i+1,:] = self.DS.fd( self.x_sol_CL[i,:] , self.u_sol_CL[i,:] , self.dt )
                    self.x_sol_OL[i+1,:] = self.DS.fd( self.x_sol_OL[i,:] , self.DS.ubar       , self.dt )
                    
                    
                # domain check
                if self.domain_check:
                    if not( self.DS.isavalidstate( x ) ):
                        self.J = self.J + self.domain_fail_cost 
                    if not( self.DS.isavalidinput( x , u ) ):
                        self.J = self.J + self.domain_fail_cost 
                    
                    
        # Final cost
        self.J = self.J + np.dot( x.T , np.dot( self.H , x ) )

            
    
    ##############################
    def plot_OL(self, show = True ):
        """ 
        Create a figure with trajectories for all states of the Open-Loop simulation
        
        """
        
        matplotlib.rc('xtick', labelsize=self.fontsize )
        matplotlib.rc('ytick', labelsize=self.fontsize )
        
        l = self.DS.n
        
        simfig , plots = plt.subplots( l , sharex=True,figsize=(4, 3),dpi=300, frameon=True)
        
        simfig.canvas.set_window_title('Open loop trajectory')
        
        
        # For all states
        for i in range( self.DS.n ):
            plots[i].plot( self.t , self.x_sol_OL[:,i] , 'b')
            plots[i].set_ylabel(self.DS.state_label[i] +'\n'+ self.DS.state_units[i] , fontsize=self.fontsize )
            plots[i].grid(True)
               
        plots[l-1].set_xlabel('Time [sec]', fontsize=self.fontsize )
        
        simfig.tight_layout()
        
        if show:
            simfig.show()
        
        self.fig   = simfig
        self.plots = plots
        
        
    ##############################
    def plot_CL(self, plot = 'All' , show = True ):
        """
        Create a figure with trajectories for all states and control inputs
        plot = 'All'
        plot = 'x'
        plot = 'u'        
        
        """
        
        matplotlib.rc('xtick', labelsize=self.fontsize )
        matplotlib.rc('ytick', labelsize=self.fontsize )
        
        # Number of subplots
        if plot == 'All':
            l = self.DS.m + self.DS.n
        elif plot == 'x':
            l = self.DS.n
        elif plot == 'u':
            l = self.DS.m
            
        simfig , plots = plt.subplots(l, sharex=True,figsize=(4, 3),dpi=300, frameon=True)
        
        simfig.canvas.set_window_title('Closed loop trajectory')
        

        j = 0 # plot index
        
        if plot == 'All' or plot == 'x':
            # For all states
            for i in range( self.DS.n ):
                plots[j].plot( self.t , self.x_sol_CL[:,i] , 'b')
                plots[j].set_ylabel(self.DS.state_label[i] +'\n'+ self.DS.state_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                j = j + 1
            
        if plot == 'All' or plot == 'u':
            # For all inputs
            for i in range( self.DS.m ):
                plots[j].plot( self.t , self.u_sol_CL[:,i] , 'r')
                plots[j].set_ylabel(self.DS.input_label[i] + '\n' + self.DS.input_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                j = j + 1
               
        plots[l-1].set_xlabel('Time [sec]', fontsize=self.fontsize )
        
        simfig.tight_layout()
        
        if show:
            simfig.show()
        
        
        self.fig   = simfig
        self.plots = plots
        
        
    #############################
    def phase_plane_trajectory(self ,  traj_CL = True, traj_OL = False , PP_CL = False , PP_OL = True ):
        """ """
        
        y1 = self.DS.axis_to_plot[0] # State for x-axis of plot
        y2 = self.DS.axis_to_plot[1] # State for y-axis of plot
        
        # Quiver
        self.PP = PhasePlot( self.DS , y1 , y2 , PP_OL , PP_CL )
        self.PP.u = self.DS.ubar
        self.PP.compute()
        self.PP.plot()
        
        #Simulation loading
        xs_OL = self.x_sol_OL
        xs_CL = self.x_sol_CL
        
        # Phase trajectory OL' 
        if traj_OL:
            plt.plot(xs_OL[:,0], xs_OL[:,1], 'b-') # path
            plt.plot([xs_OL[0,0]], [xs_OL[0,1]], 'o') # start
            plt.plot([xs_OL[-1,0]], [xs_OL[-1,1]], 's') # end
        
        # Phase trajectory CL
        if traj_CL:
            plt.plot(xs_CL[:,0], xs_CL[:,1], 'r-') # path
            plt.plot([xs_CL[-1,0]], [xs_CL[-1,1]], 's') # end
        
        plt.tight_layout()

        


'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass