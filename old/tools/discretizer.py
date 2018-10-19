# -*- coding: utf-8 -*-
"""
Created on Wed Jul 12 10:02:12 2017

@author: alxgr
"""

import numpy as np
import matplotlib.pyplot as plt


from scipy.interpolate import RectBivariateSpline as interpol2D
from scipy.interpolate import griddata

       
'''
################################################################################
'''


class GridDynamicSystem2D:
    """ Create a discrete gird state-action space for a 2D continous dynamic system, one continuous input u """
    
    ############################
    def __init__(self, sys , dt = 0.05 , x0_n = 101 , x1_n = 101 , u0_n = 11 ):
        
        self.DS = sys # Dynamic system class
        
        # Discretization Parameters
        
        # Simple 1-DoF
        self.dt    = dt        # time discretization
        self.x0_n  = x0_n      # x discretizatio
        self.x1_n  = x1_n      # dx discretization
        self.u0_n  = u0_n      # u0 discretization 
        
        # Options
        self.uselookuptable = True
        
        self.compute()  
        
    ##############################
    def compute(self):
        """  """

        self.discretizespace()
        self.discretizeactions() 
        
        print('\nDiscretization:\n---------------------------------')
        print('State space dimensions:', self.DS.n , ' Input space dimension:', self.DS.m )
        print('Number of nodes:', self.nodes_n , ' Number of actions:', self.actions_n )
        print('Number of node-action pairs:', self.nodes_n * self.actions_n )
        
        self.generate_nodes()
        self.generate_actions()
        
        if self.uselookuptable:
            self.compute_lookuptable()
            
        
    #############################
    def discretizespace(self):
        """ Grid the state space """
        
        # Grid
        self.xgriddim = ( self.x0_n , self.x1_n )
                
        self.xd       = [ None , None ]
        self.xd[0]    = np.linspace( self.DS.x_lb[0]  , self.DS.x_ub[0]  , self.x0_n  )
        self.xd[1]    = np.linspace( self.DS.x_lb[1]  , self.DS.x_ub[1]  , self.x1_n  )
        
        self.x_grid2node    = np.zeros( ( self.x0_n , self.x1_n ) , dtype = int )     # grid of corresponding index
        
        # 1-D List of nodes
        self.nodes_n        = self.x0_n * self.x1_n
        self.nodes_state    = np.zeros(( self.nodes_n , self.DS.n ), dtype = float )  # Number of nodes x state dimensions
        self.nodes_index    = np.zeros(( self.nodes_n , self.DS.n ), dtype = int   )  # Number of nodes x state dimensions
        
        
    #############################
    def discretizeactions(self):
        """ Grid the action space """
        
        # Grid
        self.ugriddim = ( self.u0_n  )
        
        self.ud       = [ None ]        
        self.ud[0]    = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , self.u0_n  )
        
        # 1-D List of actions
        self.actions_n         =   self.u0_n
        self.actions_input     =   np.zeros(( self.actions_n , self.DS.m ), dtype = float )  # Number of actions x inputs dimensions
        self.actions_index     =   np.zeros(( self.actions_n , self.DS.m ), dtype = int   )  # Number of actions x inputs dimensions
        
        
    ##############################
    def generate_nodes(self):
        """ Compute 1-D list of nodes """
        
        # For all state nodes
        node = 0
        
        for i in range(self.x0_n):
            for j in range(self.x1_n):
                
                # State
                x = np.array([ self.xd[0][i]  ,  self.xd[1][j] ])
                
                # State and grid index based on node #
                self.nodes_state[node,:] = x
                self.nodes_index[node,:] = np.array([i,j])
                
                # Node # based on index ij
                self.x_grid2node[i,j]    = node

                # Increment node number
                node = node + 1
                
                
    ##############################
    def generate_actions(self):
        """ Compute 1-D list of actions """
        
        # For all state nodes
        action = 0
        
        for k in range(self.u0_n):
                
            u = np.array([ self.ud[0][k] ])
            
            # State and grid index based on node #
            self.actions_input[action,:] = u
            self.actions_index[action,:] = k

            # Increment node number
            action = action + 1
            
            
    ##############################
    def compute_lookuptable(self):
        """ Compute lookup table for faster evaluation """

        if self.uselookuptable:
            # Evaluation lookup tables      
            self.action_isok   = np.zeros( ( self.nodes_n , self.actions_n ) , dtype = bool )
            self.x_next        = np.zeros( ( self.nodes_n , self.actions_n , self.DS.n ) , dtype = float ) # lookup table for dynamic
            
            # For all state nodes        
            for node in range( self.nodes_n ):  
                
                    x = self.nodes_state[ node , : ]
                
                    # For all control actions
                    for action in range( self.actions_n ):
                        
                        u = self.actions_input[ action , : ]
                        
                        # Compute next state for all inputs
                        x_next = self.DS.fc( x , u ) * self.dt + x
                        
                        # validity of the options
                        x_ok = self.DS.isavalidstate(x_next)
                        u_ok = self.DS.isavalidinput(x,u)
                        
                        self.x_next[ node,  action, : ] = x_next
                        self.action_isok[ node, action]        = ( u_ok & x_ok )
                
                
                
'''
################################################################################
'''


class GridDynamicSystem3D(GridDynamicSystem2D):
    """ Create a discrete gird state-action space for 3D continous dynamic system, two continuous input u """
    
    ############################
    def __init__(self, sys , dt = 0.05 , x_n = 21 ,  u_n = 11 ):
        
        self.DS = sys # Dynamic system class
        
        # Discretization Parameters
        
        # Simple 1-DoF
        self.dt    = dt        # time discretization
        self.x0_n  = x_n       # x discretizatio
        self.x1_n  = x_n       # dx discretization
        self.x2_n  = x_n       # dx discretization
        self.u0_n  = u_n      # u0 discretization
        self.u1_n  = u_n
        
        # Options
        self.uselookuptable = False # Too Big
        
        self.compute()  
            
        
    #############################
    def discretizespace(self):
        """ Grid the state space """
        
        # Grid
        self.xgriddim = ( self.x0_n , self.x1_n , self.x2_n )
                
        self.xd       = [ None , None , None ]
        self.xd[0]    = np.linspace( self.DS.x_lb[0]  , self.DS.x_ub[0]  , self.x0_n  )
        self.xd[1]    = np.linspace( self.DS.x_lb[1]  , self.DS.x_ub[1]  , self.x1_n  )
        self.xd[2]    = np.linspace( self.DS.x_lb[2]  , self.DS.x_ub[2]  , self.x2_n  )
        
        self.x_grid2node    = np.zeros( ( self.x0_n , self.x1_n , self.x2_n ) , dtype = int )     # grid of corresponding index
        
        # 1-D List of nodes
        self.nodes_n        = self.x0_n * self.x1_n * self.x2_n
        self.nodes_state    = np.zeros(( self.nodes_n , self.DS.n ), dtype = float )  # Number of nodes x state dimensions
        self.nodes_index    = np.zeros(( self.nodes_n , self.DS.n ), dtype = int   )  # Number of nodes x state dimensions
        
        
    #############################
    def discretizeactions(self):
        """ Grid the action space """
        
        # Grid
        self.ugriddim = ( self.u0_n , self.u1_n )
        
        self.ud       = [ None , None ]        
        self.ud[0]    = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , self.u0_n  )
        self.ud[1]    = np.linspace( self.DS.u_lb[1]  , self.DS.u_ub[1]  , self.u1_n  )
        
        # 1-D List of actions
        self.actions_n         =   self.u0_n * self.u1_n
        self.actions_input     =   np.zeros(( self.actions_n , self.DS.m ), dtype = float )  # Number of actions x inputs dimensions
        self.actions_index     =   np.zeros(( self.actions_n , self.DS.m ), dtype = int   )  # Number of actions x inputs dimensions
        
        
    ##############################
    def generate_nodes(self):
        """ Compute 1-D list of nodes """
        
        # For all state nodes
        node = 0
        
        for i in range(self.x0_n):
            for j in range(self.x1_n):
                for k in range(self.x2_n):
                
                    # State
                    x = np.array([ self.xd[0][i]  ,  self.xd[1][j]  , self.xd[2][k] ])
                    
                    # State and grid index based on node #
                    self.nodes_state[node,:] = x
                    self.nodes_index[node,:] = np.array([i,j,k])
                    
                    # Node # based on index ijk
                    self.x_grid2node[i,j,k]    = node
    
                    # Increment node number
                    node = node + 1
                
                
    ##############################
    def generate_actions(self):
        """ Compute 1-D list of actions """
        
        # For all state nodes
        action = 0
        
        for l in range(self.u0_n):
            for m in range(self.u1_n):
                
                u = np.array([ self.ud[0][l] , self.ud[1][m] ])
                
                # State and grid index based on node #
                self.actions_input[action,:] = u
                self.actions_index[action,:] = np.array([l,m])
    
                # Increment node number
                action = action + 1
                



'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    from AlexRobotics.dynamic import Manipulator   as M

    # Define dynamic system
    R  =  M.OneLinkManipulator()
    
    dR = GridDynamicSystem2D( R )