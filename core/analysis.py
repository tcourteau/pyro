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
##########################################################################
    
class Simulation:
    """ Simulation Class for open-loop ContinuousDynamicalSystem """
    
    
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
        
        # Output computing
        
        # Cost computing
        self.compute_cost = False
        self.J = 0
        
        
    ##############################
    def set_cost_function(self, CostFunction ):
        """ TODO """
        pass

        
    ##############################
    def compute(self):
        """ Integrate trought time """
        
        self.t  = np.linspace( self.t0 , self.tf , self.n )
        self.dt = ( self.tf + 0.0 - self.t0 ) / ( self.n - 1 )
        
        self.J  = 0
        
        if self.solver == 'ode':
        
            self.x_sol = odeint( self.cds.fo , self.x0 , self.t)   

            # Compute output values
            self.y_sol = np.zeros(( self.n , self.cds.p ))  
            
            for i in range(self.n):

                x = self.x_sol[i,:]  
                u = self.cds.ubar
                t = self.t[i]
                
                self.y_sol[i,:] = self.cds.h( x , u , t )
                
                if self.compute_cost:
                    #TODO
                    pass
                
                
        elif self.solver == 'euler':
            
            self.x_sol = np.zeros((self.n,self.cds.n))
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
                    
                if self.compute_cost:
                    #TODO
                    pass
       
        
    ##############################
    def plot(self, plot = 'x' , show = True ):
        """
        Create a figure with trajectories for all states and outputs
        plot = 'All'
        plot = 'x'
        plot = 'y'        
        """
        
        matplotlib.rc('xtick', labelsize=self.fontsize )
        matplotlib.rc('ytick', labelsize=self.fontsize )
        
        # Number of subplots
        if plot == 'All':
            l = self.cds.n + self.cds.p
        elif plot == 'x':
            l = self.cds.n
        elif plot == 'y':
            l = self.cds.p
            
        simfig , plots = plt.subplots(l, sharex=True,figsize=(4, 3),dpi=300, frameon=True)
        
        simfig.canvas.set_window_title('Open loop trajectory for ' + self.cds.name)
        
        j = 0 # plot index
        
        if plot == 'All' or plot == 'x':
            # For all states
            for i in range( self.cds.n ):
                plots[j].plot( self.t , self.x_sol[:,i] , 'b')
                plots[j].set_ylabel(self.cds.state_label[i] +'\n'+ self.cds.state_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                j = j + 1
            
        if plot == 'All' or plot == 'y':
            # For all inputs
            for i in range( self.cds.p ):
                plots[j].plot( self.t , self.y_sol[:,i] , 'r')
                plots[j].set_ylabel(self.cds.output_label[i] + '\n' + self.cds.output_units[i] , fontsize=self.fontsize )
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
               
        plt.plot(self.x_sol[:,0], self.x_sol[:,1], 'b-') # path
        plt.plot([self.x_sol[0,0]], [self.x_sol[0,1]], 'o') # start
        plt.plot([self.x_sol[-1,0]], [self.x_sol[-1,1]], 's') # end
        
        plt.tight_layout()

        


'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass