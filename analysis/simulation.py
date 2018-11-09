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


from AlexRobotics.analysis import costfunction 
from AlexRobotics.analysis import phaseanalysis

       
##################################################################### #####
# Simulation Objects
##########################################################################
    
class Simulation:
    """ 
    Simulation Class for open-loop ContinuousDynamicalSystem 
    --------------------------------------------------------
    ContinuousDynamicSystem : Instance of ContinuousDynamicSystem
    tf : final time
    n  : number of points
    solver : 'ode' or 'euler'
    """
    ############################
    def __init__(self, ContinuousDynamicSystem, tf=10, n=10001, solver='ode'):
        
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
        self.cf = costfunction.QuadraticCostFunction( ContinuousDynamicSystem )

        
    ##############################
    def compute(self):
        """ Integrate trought time """
        
        self.t  = np.linspace( self.t0 , self.tf , self.n )
        self.dt = ( self.tf + 0.0 - self.t0 ) / ( self.n - 1 )
        
        self.J  = 0
        
        if self.solver == 'ode':
        
            self.x_sol = odeint( self.cds.fbar , self.x0 , self.t)   

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
        
        # To work for both open-loop/closed-loop inputs structure
        try:
            sys = self.sys # sys is the open-loop dynamic of a closed-loop
        except:
            sys = self.cds # sys is the global system
        else:
            pass
        
        #matplotlib.rc('xtick', labelsize=self.fontsize )
        #matplotlib.rc('ytick', labelsize=self.fontsize )
        
        # Number of subplots
        if plot == 'All':
            l = sys.n + sys.m + sys.p + 2
        elif plot == 'xuj':
            l = sys.n + sys.m + 2
        elif plot == 'xu':
            l = sys.n + sys.m
        elif plot == 'xy':
            l = sys.n + sys.p
        elif plot == 'x':
            l = sys.n
        elif plot == 'u':
            l = sys.m
        elif plot == 'y':
            l = sys.p
        elif plot == 'j':
            l = 2
        else:
            raise ValueError('not a valid ploting argument')
            
        simfig , plots = plt.subplots(l, sharex=True, figsize=(4, 3), dpi=300, 
                                      frameon=True)
        
        ###############################################
        #Fix bug for single variable plotting
        if l == 1:
            plots = [plots]
        ###############################################
        
        simfig.canvas.set_window_title('Trajectory for ' + self.cds.name)
        
        j = 0 # plot index
        
        if plot=='All' or plot=='x' or plot=='xu' or plot=='xy' or plot=='xuj':
            # For all states
            for i in range( sys.n ):
                plots[j].plot( self.t , self.x_sol[:,i] , 'b')
                plots[j].set_ylabel(sys.state_label[i] +'\n'+ 
                sys.state_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                plots[j].tick_params( labelsize = self.fontsize )
                j = j + 1
                
        if plot == 'All' or plot == 'u' or plot == 'xu' or plot == 'xuj':
            # For all inputs
            for i in range( sys.m ):
                plots[j].plot( self.t , self.u_sol[:,i] , 'r')
                plots[j].set_ylabel(sys.input_label[i] + '\n' +
                sys.input_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                plots[j].tick_params( labelsize = self.fontsize )
                j = j + 1
            
        if plot == 'All' or plot == 'y' or plot == 'xy':
            # For all outputs
            for i in range( sys.p ):
                plots[j].plot( self.t , self.y_sol[:,i] , 'k')
                plots[j].set_ylabel(sys.output_label[i] + '\n' + 
                sys.output_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                plots[j].tick_params( labelsize = self.fontsize )
                j = j + 1
                
        if plot == 'All' or plot == 'j' or plot == 'xuj':
            # Cost function
            plots[j].plot( self.t , self.dJ_sol[:] , 'b')
            plots[j].set_ylabel('dJ', fontsize=self.fontsize )
            plots[j].grid(True)
            plots[j].tick_params( labelsize = self.fontsize )
            j = j + 1
            plots[j].plot( self.t , self.J_sol[:] , 'r')
            plots[j].set_ylabel('J', fontsize=self.fontsize )
            plots[j].grid(True)
            plots[j].tick_params( labelsize = self.fontsize )
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
        self.pp = phaseanalysis.PhasePlot( self.cds , x_axis , y_axis )
        self.pp.plot()
               
        plt.plot(self.x_sol[:,x_axis], self.x_sol[:,y_axis], 'b-') # path
        plt.plot([self.x_sol[0,x_axis]], [self.x_sol[0,y_axis]], 'o') # start
        plt.plot([self.x_sol[-1,x_axis]], [self.x_sol[-1,y_axis]], 's') # end
        
        plt.tight_layout()
        
        

##########################################################################
# Closed Loop Simulation
##########################################################################
    
class CLosedLoopSimulation( Simulation ):
    """ 
    Simulation Class for closed-loop ContinuousDynamicalSystem 
    --------------------------------------------------------
    CLSystem  : Instance of ClosedLoopSystem
    tf : final time
    n  : number if point
    solver : 'ode' or 'euler'
    --------------------------------------------------------
    Use this class instead of Simulation() in order to access
    internal control inputs
    """
    #######################################################################
    def __init__(self, CLSystem , tf = 10 , n = 10001 , solver = 'ode' ):
        
        Simulation.__init__(self, CLSystem , tf, n, solver) 
        
        self.sys = CLSystem.sys
        self.ctl = CLSystem.ctl
        
        # Cost computing
        self.cf = costfunction.QuadraticCostFunction( self.sys )
        
        
    ##############################
    def compute(self):
        
        Simulation.compute(self)
        
        self.compute_inputs()
        
    ##############################
    def compute_inputs(self):
        """ Compute internal control signal of the closed-loop system """
        
        self.r_sol = self.u_sol.copy() # reference is input of combined sys
        self.u_sol = np.zeros((self.n,self.sys.m))
        
        for i in range(self.n):
            
            r = self.r_sol[i,:] 
            y = self.y_sol[i,:] 
            t = self.t[i]
            
            self.u_sol[i,:] = self.ctl.c( y , r , t )
            
            
    #############################
    def phase_plane_trajectory_CL(self , x_axis , y_axis ):
        """ """
        self.pp = phaseanalysis.PhasePlot( self.cds , x_axis , y_axis )
        
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
        
        # Plot trajectory
        plt.plot(self.x_sol[:,x_axis], self.x_sol[:,y_axis], 'b-') # path
        plt.plot([self.x_sol[0,x_axis]], [self.x_sol[0,y_axis]], 'o') # start
        plt.plot([self.x_sol[-1,x_axis]], [self.x_sol[-1,y_axis]], 's') # end
        
        plt.tight_layout()
            
        
            
    

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    pass