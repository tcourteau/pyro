# -*- coding: utf-8 -*-
"""
Created on Fri Aug 07 11:51:55 2015

@author: agirard
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Embed font type in PDF
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype']  = 42
        

##########################################################################
# Simulation and Plotting Tools 
##########################################################################
       
'''
################################################################################
'''

      
        
class PhasePlot:
    """ 
    Continous dynamic system phase plot 
    
    x_axis : index of state to display as x axis
    y_axis : index of state to display as y axis
    
    """
    ############################
    def __init__(self, ContinuousDynamicSystem , x_axis = 0 ,  y_axis = 1 ):
        
        # System
        self.cds  = ContinuousDynamicSystem
        self.f    = self.cds.f      # dynamic function
        self.xbar = self.cds.xbar   # default state
        self.ubar = self.cds.ubar   # default input
        self.t    = 0               # default time
        
        # Grid
        self.x_axis = x_axis  # Index of state to plot on x axis
        self.y_axis = y_axis  # Index of state to plot on y axis
               
        self.x_axis_min = self.cds.x_lb[ self.x_axis ]
        self.x_axis_max = self.cds.x_ub[ self.x_axis ]
        self.y_axis_min = self.cds.x_lb[ self.y_axis ]
        self.y_axis_max = self.cds.x_ub[ self.y_axis ]
        
        self.x_axis_n = 21
        self.y_axis_n = 21 
        
        # Plotting params
        self.color      = 'b'        
        self.figsize    = (3, 2)
        self.dpi        = 300
        self.linewidth  = 0.005
        self.streamplot = False
        self.arrowstyle = '->'
        self.headlength = 4.5
        self.fontsize   = 6
        
    ##############################
    def compute_grid(self):
        
        x = np.linspace( self.x_axis_min , self.x_axis_max , self.x_axis_n )
        y = np.linspace( self.y_axis_min , self.y_axis_max , self.y_axis_n )
        
        self.X, self.Y = np.meshgrid( x, y)
        
        
    ##############################
    def compute_vector_field(self):
        
        self.v = np.zeros(self.X.shape)
        self.w = np.zeros(self.Y.shape)
        
        for i in range(self.x_axis_n):
            for j in range(self.y_axis_n):
                
                # Actual states
                x  = self.xbar    # default value for all states
                x[ self.x_axis ] = self.X[i, j]
                x[ self.y_axis ] = self.Y[i, j]
                
                # States derivative open loop
                dx = self.f( x , self.ubar , self.t ) 
                
                # Assign vector components
                self.v[i,j] = dx[ self.x_axis ]
                self.w[i,j] = dx[ self.y_axis ]
                       
    ##############################
    def plot_init(self):
        
        matplotlib.rc('xtick', labelsize = self.fontsize )
        matplotlib.rc('ytick', labelsize = self.fontsize ) 
        
        self.phasefig = plt.figure( figsize = self.figsize , dpi = self.dpi, frameon=True)
        self.phasefig.canvas.set_window_title('Phase plane')
        
    ##############################
    def plot_vector_field(self):
                       
        if self.streamplot:
            self.Q = plt.streamplot( self.X, self.Y, self.v, self.w, color=self.color,  linewidth = self.linewidth , arrowstyle = self.arrowstyle , arrowsize = self.headlength )
        else:
            self.Q = plt.quiver( self.X, self.Y, self.v, self.w, color=self.color,  linewidth = self.linewidth )#, headlength = self.headlength )
        
    ##############################
    def plot_finish(self):
        
        plt.xlabel(self.cds.state_label[ self.x_axis ] + ' ' + self.cds.state_units[ self.x_axis ] , fontsize = self.fontsize)
        plt.ylabel(self.cds.state_label[ self.y_axis ] + ' ' + self.cds.state_units[ self.y_axis ] , fontsize = self.fontsize)
        plt.xlim([ self.x_axis_min , self.x_axis_max ])
        plt.ylim([ self.y_axis_min , self.y_axis_max ])
        plt.grid(True)
        plt.tight_layout()
        
    ##############################
    def plot(self):
        """ Plot phase plane """
        
        self.compute_grid()
        self.plot_init()
        self.compute_vector_field()
        self.plot_vector_field()
        self.plot_finish()

       
'''
################################################################################
'''


    
class Simulation:
    """ Time simulation of a dynamic system  """
    ############################
    def __init__(self, DynamicSystem , tf = 10 , n = 10001 , solver = 'ode' ):
        
        self.cds = DynamicSystem
        self.t0 = 0
        self.tf = tf
        self.n  = int(n)
        self.dt = ( tf + 0.0 - self.t0 ) / ( n - 1 )
        
        self.x0 = np.zeros( self.cds.n )
        
        self.solver = solver
        
        # Ploting
        self.fontsize = 5
        
        # Cost computing
        self.J = 0
        self.Q = np.diag( np.ones( self.cds.n) )  # State cost per unit of time
        self.R = np.diag( np.ones( self.cds.m) )  # Input cost per unit of time
        self.H = np.diag( np.ones( self.cds.n) )  # Final State cost per unit of time
        
        self.domain_check     = False
        self.domain_fail_cost = 10
        
    ##############################
    def compute(self):
        """ Integrate trought time """
        
        self.t  = np.linspace( self.t0 , self.tf , self.n )
        self.dt = ( self.tf + 0.0 - self.t0 ) / ( self.n - 1 )
        
        self.J  = 0
        
        if self.solver == 'ode':
        
            self.x_sol_OL = odeint( self.cds.fc_OpenLoop   , self.x0 , self.t)
            self.x_sol_CL = odeint( self.cds.fc_ClosedLoop , self.x0 , self.t)
            
            # Compute control inputs
            self.u_sol_CL   = np.zeros(( self.n , self.cds.m ))  
            
            for i in range(self.n):
                
                # u & x
                u = self.cds.ctl( self.x_sol_CL[i,:] , self.t[i] )
                x = self.x_sol_CL[i,:]
                
                # integral cost
                xQx    = np.dot( x.T , np.dot( self.Q , x ) )
                uRu    = np.dot( u.T , np.dot( self.R , u ) )
                self.J = self.J + ( xQx + uRu ) * self.dt
                
                # domain check
                if self.domain_check:
                    if not( self.cds.isavalidstate( x ) ):
                        self.J = self.J + self.domain_fail_cost 
                    if not( self.cds.isavalidinput( x , u ) ):
                        self.J = self.J + self.domain_fail_cost 
                
                self.u_sol_CL[i,:] = u
                
                
        elif self.solver == 'euler':
            
            self.x_sol_OL = np.zeros((self.n,self.cds.n))
            self.x_sol_CL = np.zeros((self.n,self.cds.n))
            self.u_sol_CL = np.zeros((self.n,self.cds.m))
            
            # Initial State    
            self.x_sol_OL[0,:] = self.x0
            self.x_sol_CL[0,:] = self.x0
            
            for i in range(self.n):
                
                # u & x
                u = self.cds.ctl( self.x_sol_CL[i,:] , self.t[i] )
                x = self.x_sol_CL[i,:]
                
                # integral cost
                xQx    = np.dot( x.T , np.dot( self.Q , x ) )
                uRu    = np.dot( u.T , np.dot( self.R , u ) )
                self.J = self.J + ( xQx + uRu ) * self.dt
                
                self.u_sol_CL[i,:] = u.copy()
                
                if i+1<self.n:
                    self.x_sol_CL[i+1,:] = self.cds.fd( self.x_sol_CL[i,:] , self.u_sol_CL[i,:] , self.dt )
                    self.x_sol_OL[i+1,:] = self.cds.fd( self.x_sol_OL[i,:] , self.cds.ubar       , self.dt )
                    
                    
                # domain check
                if self.domain_check:
                    if not( self.cds.isavalidstate( x ) ):
                        self.J = self.J + self.domain_fail_cost 
                    if not( self.cds.isavalidinput( x , u ) ):
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
        
        l = self.cds.n
        
        simfig , plots = plt.subplots( l , sharex=True,figsize=(4, 3),dpi=300, frameon=True)
        
        simfig.canvas.set_window_title('Open loop trajectory')
        
        
        # For all states
        for i in range( self.cds.n ):
            plots[i].plot( self.t , self.x_sol_OL[:,i] , 'b')
            plots[i].set_ylabel(self.cds.state_label[i] +'\n'+ self.cds.state_units[i] , fontsize=self.fontsize )
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
            l = self.cds.m + self.cds.n
        elif plot == 'x':
            l = self.cds.n
        elif plot == 'u':
            l = self.cds.m
            
        simfig , plots = plt.subplots(l, sharex=True,figsize=(4, 3),dpi=300, frameon=True)
        
        simfig.canvas.set_window_title('Closed loop trajectory')
        

        j = 0 # plot index
        
        if plot == 'All' or plot == 'x':
            # For all states
            for i in range( self.cds.n ):
                plots[j].plot( self.t , self.x_sol_CL[:,i] , 'b')
                plots[j].set_ylabel(self.cds.state_label[i] +'\n'+ self.cds.state_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                j = j + 1
            
        if plot == 'All' or plot == 'u':
            # For all inputs
            for i in range( self.cds.m ):
                plots[j].plot( self.t , self.u_sol_CL[:,i] , 'r')
                plots[j].set_ylabel(self.cds.input_label[i] + '\n' + self.cds.input_units[i] , fontsize=self.fontsize )
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
        
        y1 = self.cds.axis_to_plot[0] # State for x-axis of plot
        y2 = self.cds.axis_to_plot[1] # State for y-axis of plot
        
        # Quiver
        self.PP = PhasePlot( self.cds , y1 , y2 , PP_OL , PP_CL )
        self.PP.u = self.cds.ubar
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
    
    # Default is double integrator
    cds = DynamicSystem()
    
    # Phase plane behavior with open-loop u=3, strating at [-5,-5] for 7 sec
    cds.ubar = np.array([3])
    x0      = np.array([-5,-5])
    tf      = 7
    
    cds.phase_plane_trajectory( x0 , tf )