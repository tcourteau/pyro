# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:09:12 2016

@author: alex
"""


import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from AlexRobotics.dynamic import system

'''
################################################################################
'''
class Node:
    """ node of the random tree """
    
    ############################
    def __init__(self, x , u , t , parent ):
        
        self.x = x  # Node coordinates in the state space
        self.u = u  # Control inputs used to get there
        self.t = t  # Time when arriving at x
        self.P = parent # Previous node
        
    
    ############################
    def distanceTo(self, x_other ):
        """ Compute distance to otherNode """
        
        return np.linalg.norm( self.x - x_other )
        
        

class RRT:
    """ Rapid Random Trees search algorithm """
    
    ############################
    def __init__(self, sys = system.ContinuousDynamicSystem() , x_start ):
        
        self.DS = sys          # Dynamic system class
        
        self.x_start = x_start  # origin of the graph
        
        self.start_node = Node( self.x_start , None , 0  , None )
        
        self.Nodes = []
        self.Nodes.append( self.start_node )
        
        
        # Params
        self.dt  = 0.05
        self.INF = 10000
        self.eps = 0.001
        
        self.goal_radius          = 0.2        
        self.alpha                = 0.9    # prob of random exploration
        self.max_nodes            = 25000  # maximum number of nodes
        self.max_distance_compute = 500    # maximum number of nodes to check distance
        self.max_solution_time    = 10     # won"t look for solution taking longuer than that
        
        self.test_u_domain        = False  # run a check on validity of u input at each state
        
        
        # Smoothing params
        self.low_pass_filter      = filters.low_pass( fc = 3 , dt = self.dt )        
        
        # Traj controller
        self.traj_ctl_kp          = 25
        self.traj_ctl_kd          = 10
        
        # Ploting
        self.dyna_plot            = False
        self.dyna_node_no_update  = 50
        self.fontsize             = 4
        self.y1axis               = 0  # State to plot on y1 axis
        self.y2axis               = 1  # State to plot on y2 axis
        
        self.discretizeactions()
        
        # Init
        self.solution              = None
        self.randomized_input      = False
        
        
    #############################
    def discretizeactions(self, Nu0 = 3 ):
        
        self.U = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , Nu0  )
        
        
        
        

        
        
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    pass
        