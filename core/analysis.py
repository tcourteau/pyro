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
# Phase Plot Object
##########################################################################
        
class PhasePlot:
    """ 
    Continous dynamic system phase plot 
    ---------------------------------------------------
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
        self.phasefig.canvas.set_window_title('Phase plane of ' + self.cds.name )
        
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

       
##########################################################################
# Simulation Objects
##########################################################################
    
class Simulation:
    """ 
    Simulation Class for open-loop ContinuousDynamicalSystem 
    --------------------------------------------------------
    ContinuousDynamicSystem : Instance of ContinuousDynamicSystem
    tf : final time
    n  : number if point
    solver : 'ode' or 'euler'
    """
    ############################
    def __init__(self, ContinuousDynamicSystem , tf = 10 , n = 10001 , solver = 'ode' ):
        
        self.cds = ContinuousDynamicSystem
        self.t0 = 0
        self.tf = tf
        self.n  = int(n)
        self.dt = ( tf + 0.0 - self.t0 ) / ( n - 1 )
        self.x0 = np.zeros( self.cds.n )
        self.solver = solver
        
        # Ploting
        self.fontsize = 5
        
        # Cost computing
        self.cf = QuadraticCostFunction( ContinuousDynamicSystem )
        
        
    ##############################
    def compute(self):
        """ Integrate trought time """
        
        self.t  = np.linspace( self.t0 , self.tf , self.n )
        self.dt = ( self.tf + 0.0 - self.t0 ) / ( self.n - 1 )
        
        self.J  = 0
        
        if self.solver == 'ode':
        
            self.x_sol = odeint( self.cds.fo , self.x0 , self.t)   

            # Compute inputs-output values
            self.y_sol = np.zeros(( self.n , self.cds.p ))  
            self.u_sol = np.zeros((self.n,self.cds.m))
            
            for i in range(self.n):

                x = self.x_sol[i,:]  
                u = self.cds.ubar
                t = self.t[i]
                
                self.y_sol[i,:] = self.cds.h( x , u , t )
                self.u_sol[i,:] = u
                
        elif self.solver == 'euler':
            
            self.x_sol = np.zeros((self.n,self.cds.n))
            self.u_sol = np.zeros((self.n,self.cds.m))
            self.y_sol = np.zeros((self.n,self.cds.p))
            
            # Initial State    
            self.x_sol[0,:] = self.x0
            
            for i in range(self.n):
                
                x = self.x_sol[i,:]
                u = self.cds.ubar
                t = self.t[i]
                
                if i+1<self.n:
                    self.x_sol[i+1,:] = self.cds.f( x , u , t ) * self.dt + x
                
                self.y_sol[i,:] = self.cds.h( x , u , t )
                self.u_sol[i,:] = u
                
                
    ##############################
    def compute_cost(self):
        """ Integrate cost trought time """
        
        self.J      = 0  # cost integral
        self.dJ_sol = np.zeros((self.n,1))
        self.J_sol  = np.zeros((self.n,1))
        
        for i in range(self.n):

            x = self.x_sol[i,:]  
            u = self.u_sol[i,:] 
            y = self.y_sol[i,:] 
            t = self.t[i]
            
            dJ = self.cf.g(x, u, y, t)
            self.J  = dJ * self.dt + self.J
            
            self.dJ_sol[i] = dJ
            self.J_sol[i]  = self.J
        
        # Final cost
        self.J = self.cf.h(x, y, t) + self.J
        self.J_sol[-1] = self.J
       
        
    ##############################
    def plot(self, plot = 'x' , show = True ):
        """
        Create a figure with trajectories for states, inputs, outputs and cost
        ----------------------------------------------------------------------
        plot = 'All'
        plot = 'xu'
        plot = 'xy'
        plot = 'x'
        plot = 'u'
        plot = 'y'
        plot = 'j'
        """
        
        matplotlib.rc('xtick', labelsize=self.fontsize )
        matplotlib.rc('ytick', labelsize=self.fontsize )
        
        # Number of subplots
        if plot == 'All':
            l = self.cds.n + self.cds.m + self.cds.p + 2
        elif plot == 'xuj':
            l = self.cds.n + self.cds.m + 2
        elif plot == 'xu':
            l = self.cds.n + self.cds.m
        elif plot == 'xy':
            l = self.cds.n + self.cds.p
        elif plot == 'x':
            l = self.cds.n
        elif plot == 'u':
            l = self.cds.m
        elif plot == 'y':
            l = self.cds.p
        elif plot == 'j':
            l = 2
        else:
            raise ValueError('not a valid ploting argument')
            
        simfig , plots = plt.subplots(l, sharex=True,figsize=(4, 3),dpi=300, frameon=True)
        
        #Fix bug for single variable plotting
        if l == 1:
            plots = [plots]
        
        simfig.canvas.set_window_title('Open loop trajectory for ' + self.cds.name)
        
        j = 0 # plot index
        
        if plot == 'All' or plot == 'x' or plot == 'xu' or plot == 'xy' or plot == 'xuj':
            # For all states
            for i in range( self.cds.n ):
                plots[j].plot( self.t , self.x_sol[:,i] , 'b')
                plots[j].set_ylabel(self.cds.state_label[i] +'\n'+ self.cds.state_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                j = j + 1
                
        if plot == 'All' or plot == 'u' or plot == 'xu' or plot == 'xuj':
            # For all inputs
            for i in range( self.cds.m ):
                plots[j].plot( self.t , self.u_sol[:,i] , 'r')
                plots[j].set_ylabel(self.cds.input_label[i] + '\n' + self.cds.input_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                j = j + 1
            
        if plot == 'All' or plot == 'y' or plot == 'xy':
            # For all outputs
            for i in range( self.cds.p ):
                plots[j].plot( self.t , self.y_sol[:,i] , 'k')
                plots[j].set_ylabel(self.cds.output_label[i] + '\n' + self.cds.output_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                j = j + 1
                
        if plot == 'All' or plot == 'j' or plot == 'xuj':
            # Cost function
            plots[j].plot( self.t , self.dJ_sol[:] , 'b')
            plots[j].set_ylabel('dJ', fontsize=self.fontsize )
            plots[j].grid(True)
            j = j + 1
            plots[j].plot( self.t , self.J_sol[:] , 'r')
            plots[j].set_ylabel('J', fontsize=self.fontsize )
            plots[j].grid(True)
            j = j + 1
               
        plots[l-1].set_xlabel('Time [sec]', fontsize=self.fontsize )
        
        simfig.tight_layout()
        
        if show:
            simfig.show()
        
        self.fig   = simfig
        self.plots = plots
        
        
    #############################
    def phase_plane_trajectory(self , x_axis , y_axis ):
        """ """
        self.pp = PhasePlot( self.cds , x_axis , y_axis )
        self.pp.plot()
               
        plt.plot(self.x_sol[:,x_axis], self.x_sol[:,y_axis], 'b-') # path
        plt.plot([self.x_sol[0,x_axis]], [self.x_sol[0,y_axis]], 'o') # start
        plt.plot([self.x_sol[-1,x_axis]], [self.x_sol[-1,y_axis]], 's') # end
        
        plt.tight_layout()

 
    
##########################################################################
# Cost functions
##########################################################################       

class CostFunction:
    """ 
    Mother class for cost functions of continuous dynamical systems
    ----------------------------------------------
    n : number of states
    m : number of control inputs
    p : number of outputs
    ---------------------------------------
    J = int( g(x,u,y,t) * dt ) + h( x(T) , y(T) , T )
    
    """
    ###########################################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################################
    
    ############################
    def __init__(self, ContinuousDynamicSystem ):
        
        # Parameters
        n    = ContinuousDynamicSystem.n
        m    = ContinuousDynamicSystem.m
        p    = ContinuousDynamicSystem.p
        
        raise NotImplementedError
        
    #############################
    def h(self, x , y = 0 , t = 0):
        """ Final cost function """
        
        raise NotImplementedError
    
    
    #############################
    def g(self, x , u , y = 0 , t = 0 ):
        """ step cost function """
        
        raise NotImplementedError
        

#############################################################################
     
class QuadraticCostFunction:
    """ 
    Quadratic cost functions of continuous dynamical systems
    ----------------------------------------------
    n : number of states
    m : number of control inputs
    p : number of outputs
    ---------------------------------------
    J = int( g(x,u,y,t) * dt ) + h( x(T) , y(T) , T )
    
    g = xQx + uRu + yVy
    h = 0
    
    """
    
    ############################
    def __init__(self, ContinuousDynamicSystem ):
        
        # Parameters
        n = ContinuousDynamicSystem.n
        m = ContinuousDynamicSystem.m
        p = ContinuousDynamicSystem.p
        
        # Quadratic cost weights
        self.Q = np.diag( np.ones( n ) )
        self.R = np.diag( np.ones( m ) )
        self.V = np.diag( np.zeros( p ) )
        
    #############################
    def h(self, x , y , t = 0):
        """ Final cost function with zero value """
        
        return 0
    
    
    #############################
    def g(self, x , u , y , t = 0 ):
        """ Quadratic additive cost """
        
        dJ = np.dot( x.T , np.dot(  self.Q , x ) ) + np.dot( u.T , np.dot(  self.R , u ) ) + np.dot( y.T , np.dot(  self.V , y ) )
        
        return dJ
    

##############################################################################

class TimeCostFunction:
    """ 
    Mother class for cost functions of continuous dynamical systems
    ----------------------------------------------
    n : number of states
    m : number of control inputs
    p : number of outputs
    ---------------------------------------
    J = int( g(x,u,y,t) * dt ) + h( x(T) , y(T) , T ) = T
    
    g = 1
    h = 0
    
    """
    
    ############################
    def __init__(self, ContinuousDynamicSystem ):
        
        pass
        
    #############################
    def h(self, x , y , t = 0):
        """ Final cost function with zero value """
        
        return 0
    
    
    #############################
    def g(self, x , u , y , t = 0 ):
        """ Unity """
        
        return 1

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass