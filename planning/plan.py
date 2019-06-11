# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 13:41:26 2018

@author: Alexandre
"""
###############################################################################
import numpy as np

###############################################################################
from pyro.analysis import simulation
from pyro.control  import controller
from pyro.signal   import timefiltering


###############################################################################
class OpenLoopController( controller.StaticController ) :
    """  Open-loop controller based on trajectory solution  """
    ############################
    def __init__(self, trajectory   ):
        """ """
        
        # Sys
        self.trajectory = trajectory
        
        # Dimensions
        self.k = 1   
        self.m = trajectory.m
        self.n = trajectory.n
        self.p = trajectory.n
        
        controller.StaticController.__init__(self, self.k, self.m, self.p)
        
        # Label
        self.name = 'Open Loop Controller'

    #############################
    def c( self , y , r , t  ):
        """  U depends only on time """
        
        u = self.trajectory.t2u( t )
        
        return u
        

###############################################################################
class Trajectory() :
    """    """
    ############################
    def __init__(self, x , u , t , dx = None , y = None):
        """ 
        x:  array of dim = ( time-steps , sys.n )
        u:  array of dim = ( time-steps , sys.m )
        t:  array of dim = ( time-steps , 1 )
        dx: array of dim = ( time-steps , sys.n )
        y:  array of dim = ( time-steps , sys.p )
        """
                
        self.x_sol  = x
        self.u_sol  = u
        self.t_sol  = t
        self.dx_sol = dx
        self.y_sol  = y
        
        self.compute_size()
        
    ############################
    def save(self, name = 'trajectory_solution.npy' ):
        
        data = np.array( [ self.x_sol , 
                           self.u_sol , 
                           self.t_sol ,
                           self.dx_sol,
                           self.y_sol ] )
        
        np.save( name , data )
        
        
    ############################
    def load(self, name = 'trajectory_solution.npy' ):
        
        data = np.load( name )
        
        self.x_sol  = data[0]
        self.u_sol  = data[1]
        self.t_sol  = data[2]
        self.dx_sol = data[3]
        self.y_sol  = data[4]
        
        self.compute_size()
        
    ############################
    def compute_size(self):
        
        self.time_final = self.t_sol.max()
        self.time_steps = self.t_sol.size
        
        self.n = self.x_sol.shape[1]
        self.m = self.u_sol.shape[1]
        
        self.ubar = np.zeros( self.m )
        
    
    ############################
    def low_pass_filter_x( self , fc = 3 ):
        
        # Assuming time-step is constant
        dt = self.t_sol[1] - self.t_sol[0]
        
        self.low_pass_filter = timefiltering.LowPassFilter( fc , dt )
        
        for i in range(self.n):
            self.x_sol[:,i]  = self.low_pass_filter.filter_array( self.x_sol[:,i]  )
            
        for j in range(self.m):
            self.u_sol[:,j]  = self.low_pass_filter.filter_array( self.u_sol[:,j]  )
            
        
    ############################
    def t2u(self, t ):
        """ get u from time """
        
        if t < self.time_final:
            # Find time index
            i = (np.abs(self.t_sol - t)).argmin()
            
            # Find associated control input
            u = self.u_sol[i,:]
        
        else:
            u = self.ubar
            
        return u
    
    ############################
    def t2x(self, t ):
        """ get x from time """
        
        # Find time index
        i = (np.abs(self.t_sol - t)).argmin()
        
        # Find associated control input
        x = self.x_sol[i,:]
            
        return x
    
    ############################
    def plot_trajectory(self, sys , params = 'xu' ):
        """  """
        
        sim = simulation.Simulation( sys , 
                                     self.time_final , 
                                     self.time_steps )

        sim.x_sol = self.x_sol
        sim.t     = self.t_sol
        sim.u_sol = self.u_sol
        
        sim.plot( params )
        
        sys.sim = sim



   
###############################################################################
def load_trajectory( name = 'trajectory_solution.npy' ):
    
        data = np.load( name )
        
        return Trajectory( data[0] , data[1], data[2] , data[3] , data[4] )
    

###############################################################################
def load_open_loop_controller( name = 'trajectory_solution.npy' ):
    
        traj = load_trajectory( name )
        
        return OpenLoopController( traj )
        
        
        
        
        
        
        
        
        
        