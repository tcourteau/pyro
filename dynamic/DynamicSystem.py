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

       
'''
################################################################################
'''


class DynamicSystem:
    """ 
    Mother class for dynamical systems
    ----------------------------------------------
    default value correspond to a double-integrator
    
    """
    
    ############################
    def __init__(self, n = 2 , m = 1 ):
        """ """
        
        self.n = n   # Number of states
        self.m = m   # Number of control inputs
        
        # Labels
        self.state_label = ['Position','Speed']
        self.input_label = ['Force']
        
        # Units
        self.state_units = ['[m]','[m/sec]']
        self.input_units = ['[N]']
        
        # Define the domain
        self.x_ub = np.zeros(n) + 10  # States Upper Bounds
        self.x_lb = np.zeros(n) - 10  # States Lower Bounds
        self.u_ub = np.zeros(m) + 1   # Control Upper Bounds
        self.u_lb = np.zeros(m) - 1   # Control Lower Bounds
        
        # Default State and inputs        
        self.xbar = np.zeros(n)
        self.ubar = np.zeros(m)
        
        # Ploting
        self.axis_to_plot = [0,1]  # List of state to plot on 2D graph
        
        # Init system params
        self.setparams()
        
        
    #############################
    def setparams(self):
        """ 
        Parameters initialization
        --------------------------------------
        Set model parameters here
        
        """
        
        pass
        
    
    #############################
    def fc(self, x = np.zeros(2) , u = np.zeros(1) , t = 0 ):
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
        
    ###########################################################################################
    # Basic domain checks, ovewrite if something more complex is needed, ex: obstacles, etc
    ###########################################################################################
        
        
    #############################
    def isavalidstate(self , x ):
        """ check if x is in the state domain """
        ans = False
        for i in xrange(self.n):
            ans = ans or ( x[i] < self.x_lb[i] )
            ans = ans or ( x[i] > self.x_ub[i] )
            
        return not(ans)
        
        
    #############################
    def isavalidinput(self , x , u):
        """ check if u is in the control inputs domain given x """
        ans = False
        
        for i in xrange(self.m):
            ans = ans or ( u[i] < self.u_lb[i] )
            ans = ans or ( u[i] > self.u_ub[i] )

            
        return not(ans)
        
        
    #########################################################################
    # No need to overwrite the following functions for custom dynamic systems
    #########################################################################
    
    #############################
    def fc_OpenLoop(self, x , t ):
        """ 
        Foward dynamics ( fc ) in open loop with u = uBar (default constant inputs)
        ----------------------------------------------------------------
        
        """
        
        return self.fc( x , self.ubar , t )
        
        
    #############################
    def fc_ClosedLoop(self, x , t ):
        """ 
        Foward dynamics ( fc ) in closed-loop with u = ctl( x , t)
        ----------------------------------------------------------------
        
        """
        
        # Using assigned Feedback law
        u = self.ctl( x , t )
        
        return self.fc( x , u , t )
        
    #############################
    def ctl(self, x , t ):
        """ 
        Feedback law  u = ctl( x , t )
        ----------------------------------------------
        Default is u = ubar
        
        assign controller like this
        
        MyDynamicSystem.ctl = MyFeedbackLawFunction
        
        
        """
        
        u = self.ubar
        
        return u
        
        
    #############################
    def fd(self, x = np.zeros(2) , u = np.zeros(1) , dt = 0.1 ):
        """ 
        Discrete time foward dynamics evaluation 
        -------------------------------------
        - using Euler integration
        
        """
        
        x_next = np.zeros(self.n) # k+1 State vector
        
        x_next = self.fc(x,u) * dt + x
        
        return x_next
        
        
    #############################
    def phase_plane(self , PP_CL = True , PP_OL = False ):
        """ 
        Plot Phase Plane vector field of the system
        ------------------------------------------------
        
        PP_CL : show closed-loop vector field?
        PP_OL : show open-loop vector field?
        
        """
        
        y1 = self.axis_to_plot[0] 
        y2 = self.axis_to_plot[1]
        
        # Quiver
        self.PP   = PhasePlot( self , y1 , y2 , PP_OL , PP_CL )
        
        self.PP.compute()
        self.PP.plot()
        
        plt.tight_layout()
        
        
        
    #############################
    def phase_plane_trajectory(self , x0 = [0,0,0,0] , tf = 10 , CL = True, OL = False , PP_CL = True , PP_OL = False ):
        """ 
        Simulates the system and plot the trajectory in the Phase Plane 
        ------------------------------------------------
        
        x0 : starting state
        t  : simulation time
        
        CL : show closed-loop trajectory?
        OL : show open-loop trajectory?        
        
        PP_CL : show closed-loop vector field?
        PP_OL : show open-loop vector field?
        
        """
        
        y1 = self.axis_to_plot[0] 
        y2 = self.axis_to_plot[1]
        
        # Quiver
        self.PP = PhasePlot( self , y1 , y2 , PP_OL , PP_CL )
        self.PP.u = self.ubar
        self.PP.compute()
        self.PP.plot()
        
        #Simulation
        self.Sim = Simulation( self , tf )
        self.Sim.x0 = x0
        self.Sim.compute()
        xs_OL = self.Sim.x_sol_OL
        xs_CL = self.Sim.x_sol_CL
        
        # Phase trajectory OL
        if OL:
            plt.plot(xs_OL[:,0], xs_OL[:,1], 'b-') # path
            plt.plot([xs_OL[0,0]], [xs_OL[0,1]], 'o') # start
            plt.plot([xs_OL[-1,0]], [xs_OL[-1,1]], 's') # end
        
        # Phase trajectory CL
        if CL:
            plt.plot(xs_CL[:,0], xs_CL[:,1], 'r-') # path
            plt.plot([xs_CL[-1,0]], [xs_CL[-1,1]], 's') # end
        
        plt.tight_layout()
        
        

##########################################################################
# Simulation and Plotting Tools
##########################################################################
       
'''
################################################################################
'''

      
        
class PhasePlot:
    """ 
    Dynamic system phase plot 
    
    y1axis : index of state to display as x axis
    y2axis : index of state to display as y axis
    
    CL     : show closed-loop vector field?
    OL     : show open-loop vector field?
    
    """
    ############################
    def __init__(self, DynamicSystem , y1 = 0 ,  y2 = 1 , OL = True , CL = True  ):
        
        self.DS = DynamicSystem
        
        self.f_OL  = self.DS.fc                # dynamic function
        self.f_CL  = self.DS.fc_ClosedLoop
        
        self.y1axis = y1  # State to plot on y1 axis
        self.y2axis = y2  # State to plot on y2 axis
        
        self.OL = OL
        self.CL = CL
        
        self.y1min = self.DS.x_lb[ self.y1axis ]
        self.y1max = self.DS.x_ub[ self.y1axis ]
        self.y2min = self.DS.x_lb[ self.y2axis ]
        self.y2max = self.DS.x_ub[ self.y2axis ]
        
        self.y1n = 21
        self.y2n = 21 
        
        self.x = self.DS.xbar              # Zero state
        self.u = self.DS.ubar              # Control input
        self.t = 0                         # Current time
        
        
        # Plotting params
        self.color_OL = 'b'
        self.color_CL = 'r'
        
        self.figsize    = (3, 2)
        self.dpi        = 300
        self.linewidth  = 0.005
        self.streamplot = False
        self.arrowstyle = '->'
        self.headlength = 4.5
        self.fontsize   = 6
        
        self.traj_OL    = False
        self.traj_CL    = True
        
        self.compute()
        
        
    ##############################
    def compute(self):
        """ Compute vector field """
        
        y1 = np.linspace( self.y1min , self.y1max , self.y1n )
        y2 = np.linspace( self.y2min , self.y2max , self.y2n )
        
        self.X, self.Y = np.meshgrid( y1, y2)
        
        self.v_OL, self.w_OL = np.zeros(self.X.shape), np.zeros(self.X.shape)
        self.v_CL, self.w_CL = np.zeros(self.X.shape), np.zeros(self.X.shape)
        
        for i in range(self.y1n):
            for j in range(self.y2n):
                
                # Actual states
                y1 = self.X[i, j]
                y2 = self.Y[i, j]
                x  = self.x             # default value for all states
                x[ self.y1axis ] = y1
                x[ self.y2axis ] = y2
                
                # States derivative open loop
                dx_OL = self.f_OL( x , self.u , self.t ) 
                
                # States derivative closed loop
                dx_CL = self.f_CL( x , self.t ) 
                
                # Assign vector components
                self.v_OL[i,j] = dx_OL[ self.y1axis ]
                self.w_OL[i,j] = dx_OL[ self.y2axis ]
                self.v_CL[i,j] = dx_CL[ self.y1axis ]
                self.w_CL[i,j] = dx_CL[ self.y2axis ]
                
                
                
    ##############################
    def plot(self, sim = None ):
        """ Plot phase plane """
        
        matplotlib.rc('xtick', labelsize = self.fontsize )
        matplotlib.rc('ytick', labelsize = self.fontsize ) 
        
        self.phasefig = plt.figure( figsize = self.figsize , dpi = self.dpi, frameon=True)
        
        self.phasefig.canvas.set_window_title('Phase plane')
        
        if self.OL:
            #streamplot
            if self.streamplot:
                self.Q_OL = plt.streamplot( self.X, self.Y, self.v_OL, self.w_OL, color=self.color_OL,  linewidth = self.linewidth , arrowstyle = self.arrowstyle , arrowsize = self.headlength )
            else:
                self.Q_OL = plt.quiver( self.X, self.Y, self.v_OL, self.w_OL, color=self.color_OL,  linewidth = self.linewidth )#, headlength = self.headlength )
        if self.CL:
            if self.streamplot:
                self.Q_CL = plt.streamplot( self.X, self.Y, self.v_CL, self.w_CL, color=self.color_CL ,  linewidth = self.linewidth , arrowstyle = self.arrowstyle , arrowsize = self.headlength )
            else:
                self.Q_CL = plt.quiver( self.X, self.Y, self.v_CL, self.w_CL, color=self.color_CL,  linewidth = self.linewidth , headlength = self.headlength )
        
        plt.xlabel(self.DS.state_label[ self.y1axis ] + ' ' + self.DS.state_units[ self.y1axis ] , fontsize = self.fontsize)
        plt.ylabel(self.DS.state_label[ self.y2axis ] + ' ' + self.DS.state_units[ self.y2axis ] , fontsize = self.fontsize)
        plt.xlim([ self.y1min , self.y1max ])
        plt.ylim([ self.y2min , self.y2max ])
        plt.grid(True)
        plt.tight_layout()
        
        
        # Print Sim trajectory if passed as argument
        if not( sim == None ):
            #Simulation loading
            xs_OL = sim.x_sol_OL
            xs_CL = sim.x_sol_CL
            
            # Phase trajectory OL' 
            if self.traj_OL:
                plt.plot(xs_OL[:,0], xs_OL[:,1], 'b-') # path
                plt.plot([xs_OL[0,0]], [xs_OL[0,1]], 'o') # start
                plt.plot([xs_OL[-1,0]], [xs_OL[-1,1]], 's') # end
            
            # Phase trajectory CL
            if self.traj_CL:
                plt.plot(xs_CL[:,0], xs_CL[:,1], 'r-') # path
                plt.plot([xs_CL[-1,0]], [xs_CL[-1,1]], 's') # end
        


       
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
            
            for i in xrange(self.n):
                
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
            
            for i in xrange(self.n):
                
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
        for i in xrange( self.DS.n ):
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
            for i in xrange( self.DS.n ):
                plots[j].plot( self.t , self.x_sol_CL[:,i] , 'b')
                plots[j].set_ylabel(self.DS.state_label[i] +'\n'+ self.DS.state_units[i] , fontsize=self.fontsize )
                plots[j].grid(True)
                j = j + 1
            
        if plot == 'All' or plot == 'u':
            # For all inputs
            for i in xrange( self.DS.m ):
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
    
    # Default is double integrator
    DS = DynamicSystem()
    
    # Phase plane behavior with open-loop u=3, strating at [-5,-5] for 7 sec
    DS.ubar = np.array([3])
    x0      = np.array([-5,-5])
    tf      = 7
    
    DS.phase_plane_trajectory( x0 , tf )