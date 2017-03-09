# -*- coding: utf-8 -*-
"""
Created on Mon Aug 10 13:45:44 2015

@author: agirard
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import RectBivariateSpline as interpol2D

import matplotlib.animation as animation



from AlexRobotics.dynamic import DynamicSystem as RDDS
       
'''
################################################################################
'''


class ValueIteration1DOF:
    """ Dynamic programming for 1 DOF continous dynamic system, one continuous input u """
    
    ############################
    def __init__(self, sys , cost = 'time' ):
        
        self.DS = sys # Dynamic system class
        
        # Parameters
        self.dt   = 0.05       # time discretization
        
        self.Nx0  = 101       # x discretizatio0.02**2n
        self.Nx1  = 101       # dx discretization
        self.Nu0  = 11        # u0 discretization
        
        self.INF  = 10        #  default large cost
        
        self.max_error = [0.2,0.2]  # Value of epsilon
        
        
        # Quadratic cost
        self.rho    = 1        
        self.w_quad = np.array([ 0.02 , 0.01 , self.rho * 0.01 ])
        
        
        # Predefined cost params
        if cost == 'time':
            
            print('Minimium time optimization')
            self.g    = self.g_time
            self.h    = self.h_target
            self.Nu0  = 3
            self.INF  = 6
            
        elif cost == 'quadratic':
            
            print('Quadratic cost optimization')
            self.g      = self.g_quadratic
            self.h      = self.h_zero       # no final cost
            self.Nu0    = 21
            self.INF    = 10
            
        elif cost == 'energy':
            
            print('Minimium energy optimization')
            self.g    = self.g_energy
            self.h    = self.h_target
            self.INF  = 6
        
        else :
            
            print('Warning: not a standar cost function')       
        
        
        
    #############################
    def discretizespace(self):
        """ Grid the state space """
                
        self.X = [ None , None ]
        
        self.X[0] = np.linspace( self.DS.x_lb[0]  , self.DS.x_ub[0]  , self.Nx0  )
        self.X[1] = np.linspace( self.DS.x_lb[1]  , self.DS.x_ub[1]  , self.Nx1  )
        
    #############################
    def discretizeactions(self):
        
        self.U = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , self.Nu0  )
        
        
    #############################
    def h(self,  x ):
        """ Final cost function """
        
        return 0
        
        
    #############################
    def h_zero(self,  x ):
        """ Final cost function with zero value """
        
        return 0
        
        
    #############################
    def h_target(self,  x ):
        """ Final cost function """
        # Minimum time problem h = 0 , g = 1
        
        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ):
            # On target = OK            
            cost = 0 
        else:
            # Off target = bad
            cost = self.INF
        
        return cost
        
    
    #############################
    def g(self, x , u ):
        """ step cost function """
        
        return 1
    
    #############################
    def g_time(self, x , u ):
        """ Minimum time cost """

        # On target not doing anything (don't count time at this point)
        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ) and ( abs(u[0]) <= 0.1 ):
            cost = 0
        
        # Add time for the move
        else:
            cost = self.dt # minimum time
                
        return cost

        
    #############################
    def g_quadratic(self, x , u ):
        """ Quadratic additive cost """
         # On target not doing anything (don't count time at this point)
        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ) and ( abs(u[0]) <= 0.1 ):
            cost = 0
        
        # Add time for the move
        else:
            cost = ( self.w_quad[0] * x[0] ** 2 + self.w_quad[1] * x[1] ** 2 + self.w_quad[2] * u[0] ** 2 ) * self.dt
                
        return cost
        
        
    #############################
    def g_energy(self, x , u ):
        """ Electric energy lost """

        cost = ( self.w_quad[2] * u[0] ** 2 ) * self.dt # Energy
                
        return cost
        
        
    ##############################
    def first_step(self):
        """ initial evaluation of cost-to-go """
        
        self.discretizespace()
        self.discretizeactions()
        
        self.gridsize = ( self.Nx0 , self.Nx1 )
        
        
        self.J             = np.zeros( self.gridsize )
        self.action_policy = np.zeros( self.gridsize )
        self.u0_policy     = np.zeros( self.gridsize )
        self.Jnew          = np.zeros( self.gridsize )
        self.Jplot         = np.zeros( self.gridsize )
        
        # Approximation
        self.u0_policy_a   = np.zeros( self.gridsize )
        
        
        # Evaluation lookup tables
        self.X_ok          = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0  ) )        
        self.U_ok          = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0  ) )
        self.X_next        = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0 , 2 ) ) # lookup table for dynamic
        
        
        # Initial evaluation
        
        # For all state nodes
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                
                x = np.array([ self.X[0][i]  ,  self.X[1][j] ])
                
                # Compute cost of initial states
                self.J[i,j] = self.h( x )
                
                # For all control actions
                for k in xrange( self.Nu0 ):
                    
                    u = np.array([ self.U[k] ])
                    
                    # Compute next state for all inputs
                    x_next = self.DS.fc( x , u ) * self.dt + x
                    
                    # validity of the options
                    x_ok = self.DS.isavalidstate(x_next)
                    u_ok = self.DS.isavalidinput(x,u)
                    
                    self.X_next[i,j,k,:] = x_next
                    self.U_ok[i,j,k]     = u_ok
                    self.X_ok[i,j,k]     = x_ok
                
                
    ###############################
    def compute_step(self):
        """ One step of value iteration """
        
        # Get interpolation of current cost space
        J_interpol = interpol2D( self.X[0] , self.X[1] , self.J , bbox=[None, None, None, None], kx=1, ky=1,)
        
        # For all states
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                
                # Actual state vector
                x = np.array([ self.X[0][i]  ,  self.X[1][j] ])

                # One steps costs - Q values
                Q = np.zeros( self.Nu0  ) 
                
                # For all control actions
                for k in xrange( self.Nu0 ):
                    
                    # Current u vector to test
                    u = np.array([ self.U[k] ])                 
                    
                    # Compute possibles futur states
                    x_next = self.X_next[i,j,k]  #x_next = self.DS.fc( x , u ) * self.dt + x
                                        
                    # validity of the options
                    #x_ok = self.DS.isavalidstate(x_next)
                    #u_ok = self.DS.isavalidinput(x,u)
                    x_ok = self.X_ok[i,j,k]
                    u_ok = self.U_ok[i,j,k]
                    
                    # If the current option is allowable
                    if x_ok and u_ok:
                        
                        J_next = J_interpol( x_next[0] , x_next[1] )
                        
                        # Cost-to-go of a given action
                        Q[k] = self.g( x , u ) + J_next[0,0]
                        
                    else:
                        # Not allowable states or inputs/states combinations
                        Q[k] = self.INF
                        
                        
                self.Jnew[i,j]          = Q.min()
                self.action_policy[i,j] = Q.argmin()
                self.u0_policy[i,j]     = self.U[ self.action_policy[i,j] ]
                
                # Impossible situation ( unaceptable situation for any control action )
                if self.Jnew[i,j] > (self.INF-1) :
                    self.action_policy[i,j]      = -1
                    self.u0_policy[i,j]          =  0
        
        
        # Convergence check        
        delta = self.J - self.Jnew
        j_max     =self.Jnew.max()
        delta_max = delta.max()
        delta_min = delta.min()
        print 'Max:',j_max,'Delta max:',delta_max, 'Delta min:',delta_min
        
        self.J = self.Jnew.copy()
        
        
        
    ################################
    def compute_steps(self, l = 50, plot = False):
        """ compute number of step """
               
        for i in xrange(l):
            print 'Step:',i
            self.compute_step()
            if plot:
                self.plot_J_update()
                
                
    ################################
    def plot_J(self):
        """ print graphic """
        
        xname = self.DS.state_label[0] + ' ' + self.DS.state_units[0]
        yname = self.DS.state_label[1] + ' ' + self.DS.state_units[1]
        
        self.Jplot = self.J
        
        ###################    
        
        fs = 10
        
        self.fig1 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig1.canvas.set_window_title('Cost-to-go')
        self.ax1  = self.fig1.add_subplot(1,1,1)
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im1 = plt.pcolormesh( self.X[0] , self.X[1] , self.Jplot.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()  
        
    ################################
    def plot_J_update(self):
        """ print graphic """

        self.im1.set_array( self.J )
        plt.show()
                
    
    ################################
    def plot_raw(self):
        """ print graphic """
        
        xname = self.DS.state_label[0] + ' ' + self.DS.state_units[0]
        yname = self.DS.state_label[1] + ' ' + self.DS.state_units[1]
        
        self.Jplot = self.J
        
        ###################    
        
        fs = 10
        
        self.fig1 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig1.canvas.set_window_title('Cost-to-go')
        self.ax1  = self.fig1.add_subplot(1,1,1)
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im1 = plt.pcolormesh( self.X[0] , self.X[1] , self.Jplot.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()       
        
        self.fig2 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig2.canvas.set_window_title('Optimal action index')
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im2 = plt.pcolormesh( self.X[0] , self.X[1] , self.action_policy.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()
        
        self.fig3 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig3.canvas.set_window_title('Optimal Policy for u[0]')
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im3 = plt.pcolormesh( self.X[0] , self.X[1] , self.u0_policy.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()
        
    
    ################################
    def plot_J_nice(self, maxJ = 10):
        """ print graphic """
        
        xname = self.DS.state_label[0] + ' ' + self.DS.state_units[0]
        yname = self.DS.state_label[1] + ' ' + self.DS.state_units[1]
        
        ## Saturation function for cost
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                if self.J[i,j] >= maxJ :
                    self.Jplot[i,j] = maxJ
                else:
                    self.Jplot[i,j] = self.J[i,j]
        
        ###################    
        
        fs = 10
        
        self.fig1 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig1.canvas.set_window_title('Cost-to-go')
        self.ax1  = self.fig1.add_subplot(1,1,1)
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im1 = plt.pcolormesh( self.X[0] , self.X[1] , self.Jplot.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()       
        

        
    ################################
    def assign_interpol_controller(self):
        """ controller from optimal actions """
        
        self.b_u0 = interpol2D( self.X[0] , self.X[1] , self.u0_policy , bbox=[None, None, None, None], kx=1, ky=1,)
        
        self.DS.ctl = self.feedback_law_interpol
        
        
        
    ################################
    def feedback_law_interpol(self, x , t = 0 ):
        """ controller from optimal actions """
        
        u = np.zeros( self.DS.m )
        
        u[0] = self.b_u0( x[0] , x[1] )
        
        return u
        
    ################################
    def load_data(self, name = 'DP_data'):
        """ Save optimal controller policy and cost to go """
        
        try:
            
            # Dyan prog data
            self.X              = np.load( name + '_X'  + '.npy' )
            self.J              = np.load( name + '_J'  + '.npy' )
            self.action_policy  = np.load( name + '_a'  + '.npy' )
            self.u0_policy      = np.load( name + '_u0' + '.npy' )
            
        except:
            
            print ' Failed to load DP data ' 
        
        
    ################################
    def save_data(self, name = 'DP_data'):
        """ Save optimal controller policy and cost to go """
        
        # Dyan prog data
        np.save( name + '_X'  , self.X             )
        np.save( name + '_J'  , self.J             )
        np.save( name + '_a'  , self.action_policy )
        np.save( name + '_u0' , self.u0_policy     )
        
    ################################
    def compute_traj_cost(self):
        """ Compute cost for trajectories """ 
        
        X = self.DS.Sim.x_sol_CL
        U = self.DS.Sim.u_sol_CL
        
        J_time = 0
        J_quad = 0
        J_ener = 0
                
        for i in xrange( X.shape[0] ):
            
            J_time = J_time + self.g_time(  X[i,:] , U[i,:] )        / self.dt * self.DS.Sim.tf / self.DS.Sim.n
            J_quad = J_quad + self.g_quadratic(  X[i,:] , U[i,:] )   / self.dt * self.DS.Sim.tf / self.DS.Sim.n
            J_ener = J_ener + self.g_energy(  X[i,:] , U[i,:] )      / self.dt * self.DS.Sim.tf / self.DS.Sim.n
        
        self.J_time = J_time
        self.J_quad = J_quad
        self.J_ener = J_ener
        
        print('Energy   : ' + str(J_ener))
        print('Time     : ' + str(J_time))
        print('Quadratic: ' + str(J_quad))
        
        
        
        
       
'''
################################################################################
'''


class QLearning1DOF:
    """ Dynamic programming for 1 DOF """
    
    ############################
    def __init__(self, sys , cost = 'time' , experiment_name = 'data' ):
        
        self.DS = sys # Dynamic system class
        
        # Parameters
        
        # Learning params
        self.alpha = 0.8
        self.gamma = 0.7
        self.exp_n = 0
        
        self.x0 = np.array([0,0])        
        
        #########################
        
        self.dt   = 0.1       # time discretization
        
        self.Nx0  = 51         # x discretizatio0.02**2n
        self.Nx1  = 51         # dx discretization
        self.Nu0  = 11          # u0 discretization
        
        self.INF  = 10         #  default large cost
        
        self.max_error = [0.2,0.2]  # Value of epsilon
        
        self.cost = cost
        self.experiment_name = experiment_name
        
        # Quadratic cost
        self.rho    = 0.1        
        self.w_quad = np.array([ 0.01 , 0.01 , self.rho * 0.01 ])
        
        print('Qlearning Algo:')
        
        # Predefined cost params
        if cost == 'time':
            
            print('Minimium time optimization')
            self.g    = self.g_time
            self.h    = self.h_quad
            self.Nu0  = 3
            self.INF  = 6
            
        elif cost == 'quadratic':
            
            print('Quadratic cost optimization')
            self.g      = self.g_quadratic
            self.h      = self.h_quad       # no final cost
            self.Nu0    = 3
            self.INF    = 10
            
        elif cost == 'energy':
            
            print('Minimium energy optimization')
            self.g    = self.g_energy
            self.h    = self.h_target
            self.INF  = 6
        
        else :
            
            print('Warning: not a standar cost function')        
        
        
        
    #############################
    def discretizespace(self):
        """ Grid the state space """
                
        self.X = [ None , None ]
        
        self.X[0] = np.linspace( self.DS.x_lb[0]  , self.DS.x_ub[0]  , self.Nx0  )
        self.X[1] = np.linspace( self.DS.x_lb[1]  , self.DS.x_ub[1]  , self.Nx1  )
        
    #############################
    def discretizeactions(self):
                
        self.U  = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , self.Nu0  )
        
        
    #############################
    def h(self,  x ):
        """ Final cost function """
        
        return 0
        
        
    #############################
    def h_zero(self,  x ):
        """ Final cost function with zero value """
        
        return 0
        
        
    #############################
    def h_target(self,  x ):
        """ Final cost function """
        # Minimum time problem h = 0 , g = 1
        
        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ):
            # On target = OK            
            cost = 0 
        else:
            # Off target = bad
            cost = self.INF
        
        return cost
        
    #############################
    def h_quad(self,  x ):
        """ Final cost function """
        # Minimum time problem h = 0 , g = 1
        
        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ):
            # On target = OK            
            cost = 0 
        else:
            # Off target = bad
            cost =  ( self.w_quad[0] * x[0] ** 2 + self.w_quad[1] * x[1] ** 2 ) * 10
        
        return cost
        
    
    #############################
    def g(self, x , u ):
        """ step cost function """
        
        return 1
    
    #############################
    def g_time(self, x , u ):
        """ Minimum time cost """

        # On target not doing anything (don't count time at this point)
        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ) and ( abs(u[0]) <= 0.1 ):
            cost = 0
        
        # Add time for the move
        else:
            cost = self.dt # minimum time
                
        return cost

        
    #############################
    def g_quadratic(self, x , u ):
        """ Quadratic additive cost """
         # On target not doing anything (don't count time at this point)
        if ( abs(x[1]) <= self.max_error[1] ) and ( abs(x[0]) <= self.max_error[0] ) and ( abs(u[0]) <= 0.1 ):
            cost = 0
        
        # Add time for the move
        else:
            cost = ( self.w_quad[0] * x[0] ** 2 + self.w_quad[1] * x[1] ** 2 + self.w_quad[2] * u[0] ** 2 ) * self.dt
                
        return cost
        
        
    #############################
    def g_energy(self, x , u ):
        """ Electric energy lost """

        cost = ( self.w_quad[2] * u[0] ** 2 ) * self.dt # Energy
                
        return cost
        
        
    ##############################
    def first_step(self):
        """ initial evaluation of cost-to-go """
        
        self.discretizespace()
        self.discretizeactions()
        
        self.gridsize = ( self.Nx0 , self.Nx1 )
        
        self.J             = np.zeros( self.gridsize )
        self.action_policy = np.zeros( self.gridsize )
        self.u0_policy     = np.zeros( self.gridsize )
        self.Jnew          = np.zeros( self.gridsize )
        self.Jplot         = np.zeros( self.gridsize )
        
        # Approximation
        self.u0_policy_a   = np.zeros( self.gridsize )
        
        
        # Evaluation lookup tables
        self.X_ok          = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0 ) )        
        self.U_ok          = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0 ) )
        self.X_next        = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0 , 2 ) ) # lookup table for dynamic
        
        # Q-values
        self.Q            = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0 ) )
        
        
        # Initial evaluation
        
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                
                x = np.array([ self.X[0][i]  ,  self.X[1][j] ])
                
                # Compute cost of initial states
                self.J[i,j] = self.h( x )
                
                for k in xrange( self.Nu0 ):
                    
                    self.Q[i,j,k]  = self.J[i,j] # Initial Q-value is only local cost
                    
                    u = self.U[k]
                    
                    if self.DS.m == 1:
                        u = np.array( [ u ] )
                    
                    # Compute next state for all inputs
                    x_next = self.DS.fc( x , u ) * self.dt + x
                    
                    # validity of the options
                    x_ok = self.DS.isavalidstate( x_next )
                    u_ok = self.DS.isavalidinput( x , u  )
                    
                    self.X_next[i,j,k,:] = x_next
                    self.U_ok[i,j,k]     = u_ok
                    self.X_ok[i,j,k]     = x_ok
                    
        self.assign_interpol_controller()
        
        
                    
    ##############################
    def Qlearn(self,i,j,k):
        """  """
        J_interpol = interpol2D( self.X[0] , self.X[1] , self.J , bbox=[None, None, None, None], kx=1, ky=1,)
        
        x      = np.array([ self.X[0][i]  ,  self.X[1][j] ])
        u      = self.U[k]        
        x_next = self.X_next[i,j,k]
        
        x_ok = self.X_ok[i,j,k]
        u_ok = self.U_ok[i,j,k]
        
        if self.DS.m ==1:
            u = np.array( [u] )
        
        # New Q sample

        if x_ok and u_ok:
            J_next = J_interpol( x_next[0] , x_next[1] )
            Q_sample = self.g( x , u ) + J_next[0,0]
        else:
            Q_sample = self.INF
        
        # Q update
        error          = Q_sample      - self.Q[i,j,k]
        self.Q[i,j,k]  = self.Q[i,j,k] + self.alpha * error
        
        # J and Policy update        
        Q_list                  = self.Q[i,j,:]        
        self.J[i,j]             = Q_list.min()
        self.action_policy[i,j] = Q_list.argmin()
        self.u0_policy[i,j]     = self.U[ self.action_policy[i,j] ]
                
        # Impossible situation
        if self.J[i,j] > (self.INF-1) :
            self.action_policy[i,j]      = -1
            self.u0_policy[i,j]          =  0
            
        
                
    ##############################
    def Qlearn2( self , x = np.array([0,0]) , k = 0 ):
        """  """
        
        i = (np.abs(self.X[0]-x[0])).argmin()
        j = (np.abs(self.X[1]-x[1])).argmin()
        
        self.Qlearn(i,j,k)
        
    ##############################
    def Qlearn3( self , x = np.array([0,0]) , u = np.array([0]) ):
        """ Find closest index before calling Qlearn """
        
        i = (np.abs(self.X[0]-x[0])).argmin()
        j = (np.abs(self.X[1]-x[1])).argmin()
        k =  np.abs( self.U - u[0] ).argmin()
        
        self.Qlearn(i,j,k)
        
    ##############################
    def exploration_ctl( self , x = np.array([0,0]) , t = 0 ):
        """ Random or Optimal CTL """
        
        u = np.zeros( self.DS.m  )
        
        
        if np.random.uniform(0,1) < self.gamma:
            
            # Current optimal behavior
            u[0] = self.feedback_law_interpol( x , t )
        
        else:
            
            # Random exploration
            random_index = int(np.random.uniform( 0 , self.Nu0 ))
            u[0] = self.U[ random_index ]         
            
        return u
        
        
    ##############################
    def training( self ,  n_trial = 1 , random = False , show = True ):
        """ Training experiments """
        
        x0 = self.x0
        tf = 10 
        
        dt      = 0.05
        plot    = False
        n_plot  = 1000.
        n_print = 100.
        
        for i in xrange( n_trial ):
            
            self.exp_n = self.exp_n + 1

            if random:
                p = np.random.uniform( x0[0] - 1 , x0[0] + 1 )
                s = np.random.uniform( x0[1] - 0.5 , x0[1] + 0.5 )
                x = np.array([p,s])
                
            else:
                x = x0
            
            if (i/n_print-int(i/n_print)) < 0.00001 :
                print 'Experiment #',self.exp_n
            
            if (i/n_plot-int(i/n_plot)) < 0.00001 and show :
            # Show behavior so far

                plot = True
                self.DS.ctl  = self.feedback_law_interpol
                
                self.save_data( self.experiment_name )
                    
                
            else:
                plot = False
                self.DS.ctl  = self.exploration_ctl
            
            
            self.experiment( x , tf , dt ,  plot )
            
            # Update optimal laws
            self.assign_interpol_controller()
            
            
    #############################
    def experiment(self, x0 = np.array([0,0]) , tf = 10 , dt = 0.05 , plot = False ):
        """ Simulate (EULER) and animate robot """
        
        n  = int( ( tf + 0.0 ) / dt + 1 )
        
        self.DS.Sim = RDDS.Simulation( self.DS , tf , n , 'euler' )
        
        self.DS.Sim.x0 = x0
        
        self.DS.Sim.compute()
        
        if plot:
            self.DS.PTS = np.zeros((2,2,n))
            
            for i in xrange(n):
                            
                self.DS.PTS[:,:,i] = self.DS.fwd_kinematic( self.DS.Sim.x_sol_CL[i,0] ) # Forward kinematic

            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
            self.ax.grid()
            
            self.DS.line, = self.ax.plot([], [], 'o-', lw=2)
            self.DS.time_template = 'time = %.1fs'
            self.DS.time_text = self.ax.text(0.05, 0.9, '', transform=self.ax.transAxes)
            self.DS.ani = animation.FuncAnimation( self.fig, self.DS.__animateStop__, n, interval=25, blit=True, init_func=self.DS.__ani_init__ , repeat=False)
        
            plt.show()
            
        #Learning
        
        for i in xrange(n):      
        
            self.Qlearn3( self.DS.Sim.x_sol_CL[i,:] ,  self.DS.Sim.u_sol_CL[i,:] )
            
        
    ###############################
    def compute_step(self):
        """ One step of value iteration """
        
        # Get interpolation of current cost space
        J_interpol = interpol2D( self.X[0] , self.X[1] , self.J , bbox=[None, None, None, None], kx=1, ky=1,)
        
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                
                # Actual state vector
                x = np.array([ self.X[0][i]  ,  self.X[1][j] ])
                #print x
                
                # One steps costs
                C = np.zeros( self.Nu0 * 2 ) 
                
                for k in xrange( self.Nu0 * 2 ):
                    
                    # Current u vector to test
                    u = self.U[k]                    
                    
                    # Compute possibles futur states
                    x_next = self.X_next[i,j,k]  #x_next = self.DS.fc( x , u ) * self.dt + x
                                        
                    # validity of the options
                    #x_ok = self.DS.isavalidstate(x_next)
                    #u_ok = self.DS.isavalidinput(x,u)
                    x_ok = self.X_ok[i,j,k]
                    u_ok = self.U_ok[i,j,k]
                    
                    # If the current option is allowable
                    if x_ok and u_ok:
                        
                        J_next = J_interpol( x_next[0] , x_next[1] )
                        
                        # Cost-to-go of a given action
                        C[k] = self.g( x , u ) + J_next[0,0]
                        
                    else:
                        # Not allowable states or inputs/states combinations
                        C[k] = self.INF
                        
                    #print x,u,x_next,C[k]
                        
                self.Jnew[i,j]          = C.min()
                self.action_policy[i,j] = C.argmin()
                self.u0_policy[i,j]     = self.U[ self.action_policy[i,j] ]
                
                # Impossible situation
                if self.Jnew[i,j] > (self.INF-1) :
                    self.action_policy[i,j]      = -1
                    self.u0_policy[i,j]          =  0
                
        
        delta = self.J - self.Jnew
        j_max     = self.Jnew.max()
        delta_max = delta.max()
        delta_min = delta.min()
        print 'Max:',j_max,'Delta max:',delta_max, 'Delta min:',delta_min
        
        self.J = self.Jnew.copy()
        
        
        
        
        
    ################################
    def compute_steps(self, l = 50, plot = False):
        """ compute number of step """
        
        #self.first_step()
        #self.plot_J()
        
        for i in xrange(l):
            print 'Step:',i
            self.compute_step()
            if plot:
                self.plot_J_update()
                
                
    ################################
    def plot_J(self):
        """ print graphic """
        
        xname = self.DS.state_label[0] + ' ' + self.DS.state_units[0]
        yname = self.DS.state_label[1] + ' ' + self.DS.state_units[1]
        
        self.Jplot = self.J
        
        ###################    
        
        fs = 10
        
        self.fig1 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig1.canvas.set_window_title('Cost-to-go')
        self.ax1  = self.fig1.add_subplot(1,1,1)
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im1 = plt.pcolormesh( self.X[0] , self.X[1] , self.Jplot.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()  
        
    
    ################################
    def plot_raw_nice(self, maxJ = 10):
        """ print graphic """
        
        xname = self.DS.state_label[0] + ' ' + self.DS.state_units[0]
        yname = self.DS.state_label[1] + ' ' + self.DS.state_units[1]
        
        ## Saturation function for cost
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                if self.J[i,j] >= maxJ :
                    self.Jplot[i,j] = maxJ
                else:
                    self.Jplot[i,j] = self.J[i,j]
        
        ###################    
        
        fs = 10
        
        self.fig1 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig1.canvas.set_window_title('Cost-to-go')
        self.ax1  = self.fig1.add_subplot(1,1,1)
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im1 = plt.pcolormesh( self.X[0] , self.X[1] , self.Jplot.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()       
        
        self.fig2 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig2.canvas.set_window_title('Optimal action index')
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im2 = plt.pcolormesh( self.X[0] , self.X[1] , self.action_policy.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()
        
        self.fig3 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig3.canvas.set_window_title('Optimal Policy for u[0]')
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im3 = plt.pcolormesh( self.X[0] , self.X[1] , self.u0_policy.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()      
        

        
    ################################
    def assign_interpol_controller(self):
        """ controller from optimal actions """
        
        self.b_u0 = interpol2D( self.X[0] , self.X[1] , self.u0_policy , bbox=[None, None, None, None], kx=1, ky=1,)
        
        
        self.DS.ctl = self.feedback_law_interpol
        
        
        
    ################################
    def feedback_law_interpol(self, x , t = 0 ):
        """ controller from optimal actions """
        
        u = np.zeros( self.DS.m )
        
        u[0] = self.b_u0( x[0] , x[1] )
        
        return u
        
    ################################
    def load_data(self, name = 'DP_data'):
        """ Save optimal controller policy and cost to go """
        
        folder = 'data/'
        
        # Dyan prog data
        self.X              = np.load( folder + name + '_X'  + '.npy' )
        self.J              = np.load( folder + name + '_J'  + '.npy' )
        self.action_policy  = np.load( folder + name + '_a'  + '.npy' )
        self.u0_policy      = np.load( folder + name + '_u0' + '.npy' )
        self.Q              = np.load( folder + name + '_Q'  + '.npy' )
        
        self.assign_interpol_controller()
        
        
    ################################
    def save_data(self, name = 'DP_data'):
        """ Save optimal controller policy and cost to go """
        
        folder = 'data/'
        
        # Dyan prog data
        np.save( folder + name + '_X'  , self.X             )
        np.save( folder + name + '_J'  , self.J             )
        np.save( folder + name + '_a'  , self.action_policy )
        np.save( folder + name + '_u0' , self.u0_policy     )
        np.save( folder + name + '_Q'  , self.Q             )



       
'''
################################################################################
'''


class ValueIteration_hybrid_1DOF( ValueIteration1DOF ) : 


    ############################
    def __init__( self , sys , cost = 'time'  )    :
        
        ValueIteration1DOF.__init__( self, sys , cost )
        
        
        
    #############################
    def discretizeactions(self):
        
        self.U = np.zeros([self.Nu0 * 2 , 2])
        
        # Continuous options
        Uc   = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , self.Nu0  )
        self.U[0:self.Nu0,0]  = Uc
        self.U[self.Nu0:,0]   = Uc
        
        # Discrete options
        self.U[0:self.Nu0,1]  = 1 # Gear #1
        self.U[self.Nu0:,1]   = 10 # Gear #2
        
        
        
    ##############################
    def first_step(self):
        """ initial evaluation of cost-to-go """
        
        self.discretizespace()
        self.discretizeactions()
        
        self.gridsize = ( self.Nx0 , self.Nx1 )
        
        
        self.J             = np.zeros( self.gridsize )
        self.action_policy = np.zeros( self.gridsize )
        self.u0_policy     = np.zeros( self.gridsize )
        self.u1_policy     = np.zeros( self.gridsize )
        self.Jnew          = np.zeros( self.gridsize )
        self.Jplot         = np.zeros( self.gridsize )
        
        # Approximation
        self.u0_policy_a   = np.zeros( self.gridsize )
        self.u1_policy_a   = np.zeros( self.gridsize )
        
        
        # Evaluation lookup tables
        self.X_ok          = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0 * 2 ) )        
        self.U_ok          = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0 * 2 ) )
        self.X_next        = np.zeros( ( self.Nx0 , self.Nx1 , self.Nu0 * 2 , 2 ) ) # lookup table for dynamic
        
        
        # Initial evaluation
        
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                
                x = np.array([ self.X[0][i]  ,  self.X[1][j] ])
                
                # Compute cost of initial states
                self.J[i,j] = self.h( x )
                
                for k in xrange( self.Nu0 * 2 ):
                    
                    u = self.U[k] 
                    
                    # Compute next state for all inputs
                    x_next = self.DS.fc( x , u ) * self.dt + x
                    
                    # validity of the options
                    x_ok = self.DS.isavalidstate(x_next)
                    u_ok = self.DS.isavalidinput(x,u)
                    
                    self.X_next[i,j,k,:] = x_next
                    self.U_ok[i,j,k]     = u_ok
                    self.X_ok[i,j,k]     = x_ok
                    
                    
                    
    ###############################
    def compute_step(self):
        """ One step of value iteration """
        
        # Get interpolation of current cost space
        J_interpol = interpol2D( self.X[0] , self.X[1] , self.J , bbox=[None, None, None, None], kx=1, ky=1,)
        
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                
                # Actual state vector
                x = np.array([ self.X[0][i]  ,  self.X[1][j] ])
                #print x
                
                # One steps costs
                Q = np.zeros( self.Nu0 * 2 ) 
                
                for k in xrange( self.Nu0 * 2 ):
                    
                    # Current u vector to test
                    u =  self.U[k]            
                    
                    # Compute possibles futur states
                    x_next = self.X_next[i,j,k]  #x_next = self.DS.fc( x , u ) * self.dt + x
                                        
                    # validity of the options
                    #x_ok = self.DS.isavalidstate(x_next)
                    #u_ok = self.DS.isavalidinput(x,u)
                    x_ok = self.X_ok[i,j,k]
                    u_ok = self.U_ok[i,j,k]
                    
                    # If the current option is allowable
                    if x_ok and u_ok:
                        
                        J_next = J_interpol( x_next[0] , x_next[1] )
                        
                        # Cost-to-go of a given action
                        #print x,u,self.g( x , u ) , J_next[0,0]
                        Q[k] = self.g( x , u ) + J_next[0,0]
                        
                    else:
                        # Not allowable states or inputs/states combinations
                        Q[k] = self.INF
                        
                    #print x,u,x_next,C[k]
                        
                self.Jnew[i,j]          = Q.min()
                self.action_policy[i,j] = Q.argmin()
                self.u0_policy[i,j]     = self.U[ self.action_policy[i,j] ][0]
                self.u1_policy[i,j]     = self.U[ self.action_policy[i,j] ][1]
                
                # Impossible situation
                if self.Jnew[i,j] > (self.INF-1) :
                    self.action_policy[i,j]      = -1
                    self.u0_policy[i,j]          =  0
                    self.u1_policy[i,j]          =  1
                
        
        delta = self.J - self.Jnew
        j_max     =self.Jnew.max()
        delta_max = delta.max()
        delta_min = delta.min()
        print 'Max:',j_max,'Delta max:',delta_max, 'Delta min:',delta_min
        
        self.J = self.Jnew.copy()
        
        
    ################################
    def assign_interpol_controller(self):
        """ controller from optimal actions """
        
        self.b_u0 = interpol2D( self.X[0] , self.X[1] , self.u0_policy , bbox=[None, None, None, None], kx=1, ky=1,)
        self.b_u1 = interpol2D( self.X[0] , self.X[1] , self.u1_policy , bbox=[None, None, None, None], kx=1, ky=1,)
        
        self.DS.ctl = self.feedback_law_interpol
        
        
        
    ################################
    def feedback_law_interpol(self, x , t = 0 ):
        """ controller from optimal actions """
        
        u = np.zeros( self.DS.m )
        
        u[0] = self.b_u0( x[0] , x[1] )
        u[1] = np.round( self.b_u1( x[0] , x[1] ) )
        
        return u
        
        
    ################################
    def load_data(self, name = 'DP_data'):
        """ Save optimal controller policy and cost to go """
        
        # Dyan prog data
        self.X              = np.load( name + '_X'  + '.npy' )
        self.J              = np.load( name + '_J'  + '.npy' )
        self.action_policy  = np.load( name + '_a'  + '.npy' )
        self.u0_policy      = np.load( name + '_u0' + '.npy' )
        self.u1_policy      = np.load( name + '_u1' + '.npy' )
        
        self.assign_interpol_controller()
        
        
        
    ################################
    def save_data(self, name = 'DP_data'):
        """ Save optimal controller policy and cost to go """
        
        # Dyan prog data
        np.save( name + '_X'  , self.X             )
        np.save( name + '_J'  , self.J             )
        np.save( name + '_a'  , self.action_policy )
        np.save( name + '_u0' , self.u0_policy     )
        np.save( name + '_u1' , self.u1_policy     )
        
        
        
        
    ################################
    def plot_raw_nice(self, maxJ = 10):
        """ print graphic """
        
        xname = self.DS.state_label[0] + ' ' + self.DS.state_units[0]
        yname = self.DS.state_label[1] + ' ' + self.DS.state_units[1]
        
        ## Saturation function for cost
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                if self.J[i,j] >= maxJ :
                    self.Jplot[i,j] = maxJ
                else:
                    self.Jplot[i,j] = self.J[i,j]
        
        ###################    
        
        fs = 10
        
        self.fig1 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig1.canvas.set_window_title('Cost-to-go')
        self.ax1  = self.fig1.add_subplot(1,1,1)
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im1 = plt.pcolormesh( self.X[0] , self.X[1] , self.Jplot.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()       
        
        self.fig2 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig2.canvas.set_window_title('Optimal action index')
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im2 = plt.pcolormesh( self.X[0] , self.X[1] , self.action_policy.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()
        
        self.fig3 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig3.canvas.set_window_title('Optimal Policy for u[0]')
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im3 = plt.pcolormesh( self.X[0] , self.X[1] , self.u0_policy.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()
        
        self.fig4 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig4.canvas.set_window_title('Optimal Policy for u[1]')
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im4 = plt.pcolormesh( self.X[0] , self.X[1] , self.u1_policy.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()
        
        
    
        
        
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    pass
    