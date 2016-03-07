# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:09:12 2016

@author: alex
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

'''
################################################################################
'''


class RRT:
    """ Rapid Random Trees search algorithm """
    
    ############################
    def __init__(self, sys , x_start ):
        
        self.DS = sys          # Dynamic system class
        
        self.start = x_start  # origin of the graph
        
        self.Nodes = []
        self.Nodes.append( Node( self.start , sys.ubar  , None ) )
        
        # Params
        self.dt  = 0.1
        self.INF = 10000
        self.eps = 0.05
        
        self.discretizeactions()
        
        
        #############################
    def discretizeactions(self, Nu0 = 3 ):
        
        self.U = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , Nu0  )
        
        
    ############################
    def rand_state(self):    
        """ Sample a random state """
        
        ranges = self.DS.x_ub - self.DS.x_lb
        
        x_random = np.random.rand( self.DS.n ) * ranges + self.DS.x_lb
        
        return x_random
        
        
    ############################
    def nearest_neighbor(self, x_target ):    
        """ Sample a random state """
        
        closest_node = None
        min_distance = self.INF
        
        for node in self.Nodes:
            d = node.distanceTo( x_target )
            if d < min_distance:
                min_distance = d
                closest_node = node
                
        return closest_node
        
        
    ############################
    def select_control_input(self, x_target , closest_node ):    
        """ Sample a random state """
        
        new_node     = None
        min_distance = self.INF
        
        for u in self.U:
            if self.DS.m == 1:
                u = np.array([u])
            x_next = closest_node.x + self.DS.fc( closest_node.x , u ) * self.dt
            node   = Node( x_next , u , closest_node )
            d = node.distanceTo( x_target )
            if d < min_distance:
                min_distance = d
                new_node     = node
                
        return new_node
        
    
    ############################
    def one_step(self):    
        """ """
        x_random  = self.rand_state()
        node_near = self.nearest_neighbor( x_random )
        new_node  = self.select_control_input( x_random , node_near )
        
        self.Nodes.append( new_node )
        
        
    ############################
    def compute_steps(self , n , plot = False ):    
        """ """
        for i in xrange( n ):
            self.one_step()
            
        if plot:
            self.plot_2D_Tree()
    
    
    ############################
    def plot_2D_Tree(self):
        """ """
        
        self.y1axis = 0  # State to plot on y1 axis
        self.y2axis = 1  # State to plot on y2 axis
        
        self.y1min = self.DS.x_lb[ self.y1axis ]
        self.y1max = self.DS.x_ub[ self.y1axis ]
        self.y2min = self.DS.x_lb[ self.y2axis ]
        self.y2max = self.DS.x_ub[ self.y2axis ]
        
        self.phasefig = plt.figure(figsize=(3, 2),dpi=300, frameon=True)
        self.ax       = self.phasefig.add_subplot(111)
        
        for node in self.Nodes:
            if not(node.P==None):
                line = self.ax.plot( [node.x[0],node.P.x[0]] , [node.x[1],node.P.x[1]] , 'o-')
        
        
        plt.xlabel(self.DS.state_label[ self.y1axis ] + ' ' + self.DS.state_units[ self.y1axis ] , fontsize=6)
        plt.ylabel(self.DS.state_label[ self.y2axis ] + ' ' + self.DS.state_units[ self.y2axis ] , fontsize=6)
        plt.xlim([ self.y1min , self.y1max ])
        plt.ylim([ self.y2min , self.y2max ])
        plt.grid(True)
        plt.tight_layout()
        plt.show()
        
    
    
        
        
class Node:
    """ node of the graph """
    
    ############################
    def __init__(self, x , u , parent ):
        
        self.x = x  # Node coordinates in the state space
        self.u = u  # Control inputs used to get there
        self.P = parent # Previous node
        
    
    ############################
    def distanceTo(self, x_other ):
        """ Compute distance to otherNode """
        
        return np.linalg.norm( self.x - x_other )
        
        
        
        
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    pass
        
        
