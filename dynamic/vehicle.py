# -*- coding: utf-8 -*-
"""
Created on Tue Oct 23 20:45:37 2018

@author: Alexandre
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from AlexRobotics.dynamic import system
from AlexRobotics.analysis import graphical


##############################################################################
        
class KinematicBicyleModel( system.ContinuousDynamicSystem ):
    """ 
    
    dx   = V cos ( phi )
    dy   = V sin ( phi )
    dphi = V/l tan ( beta )
    
    """
    
    ############################
    def __init__(self):
        """ """
    
        # Dimensions
        self.n = 3   
        self.m = 2   
        self.p = 3
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
        
        # Labels
        self.name = 'Kinematic Bicyle Model'
        self.state_label = ['x','y','phi']
        self.input_label = ['v', 'beta']
        self.output_label = ['x','y','phi']
        
        # Units
        self.state_units = ['[m]','[m]','[rad]']
        self.input_units = ['[m/sec]', '[rad]']
        self.output_units = ['[m]','[m]','[rad]']
        
        # State working range
        self.x_ub = np.array([+5,+2,+3.14])
        self.x_lb = np.array([-5,-2,-3.14])
        
        # Model param
        self.lenght = 1
        
        # Graphic output parameters 
        self.dynamic_domain  = True
        
    #############################
    def f(self, x = np.zeros(3) , u = np.zeros(2) , t = 0 ):
        """ 
        Continuous time foward dynamics evaluation
        
        dx = f(x,u,t)
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        dx = np.zeros(self.n) # State derivative vector

        dx[0] = u[0] * np.cos( x[2] )
        dx[1] = u[0] * np.sin( x[2] )
        dx[2] = u[0] * np.tan( u[1] ) * ( 1. / self.lenght) 
        
        return dx
    
    
    ###########################################################################
    # For graphical output
    ###########################################################################
    
    
    #############################
    def xut2q( self, x , u , t ):
        """ compute config q """
        
        q   = np.append(  x , u[1] ) # steering angle is part of the config
        
        return q
    
    ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 10
        
        x = q[0]
        y = q[1]
        z = 0
        
        if self.dynamic_domain:
        
            domain  = [ ( -l + x , l + x ) ,
                        ( -l + y , l + y ) ,
                        ( -l + z , l + z ) ]#  
        else:
            
            domain  = [ ( -l , l ) ,
                        ( -l , l ) ,
                        ( -l , l ) ]#
            
                
        return domain
    
    ###########################################################################
    def forward_kinematic_lines(self, q ):
        """ 
        Compute points p = [x;y;z] positions given config q 
        ----------------------------------------------------
        - points of interest for ploting
        
        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines
        
        """
        
        lines_pts = [] # list of array (n_pts x 3) for each lines
        
        ###########################
        # Top line
        ###########################
            
        pts = np.zeros((2,3))
        
        pts[0,0] = -10000
        pts[0,1] = 1
        pts[1,0] = 10000
        pts[1,1] = 1
        
        lines_pts.append( pts )
        
        ###########################
        # bottom line
        ###########################
        
        pts = np.zeros((2,3))
        
        pts[0,0] = -10000
        pts[0,1] = -1
        pts[1,0] = 10000
        pts[1,1] = -1
        
        lines_pts.append( pts )
        
        ###########################
        # Car
        ###########################
        
        pts = np.zeros((3,3))
        
        pts[0,0] = q[0]
        pts[0,1] = q[1]
        pts[1,0] = q[0] + self.lenght * np.cos( q[2] )
        pts[1,1] = q[1] + self.lenght * np.sin( q[2] )
        pts[2,0] = ( q[0] + self.lenght * np.cos( q[2] ) + 
                       0.2 * self.lenght * np.cos( q[2] + q[3] ) )
        pts[2,1] = ( q[1] + self.lenght * np.sin( q[2] ) + 
                       0.2 * self.lenght * np.sin( q[2] + q[3] ) )
        
        lines_pts.append( pts )
                
        return lines_pts
    
    

##############################################################################
        
class HolonomicMobileRobot( system.ContinuousDynamicSystem ):
    """ 
    
    dx   = u[0]
    dy   = u[1]
    
    """
    
    ############################
    def __init__(self):
        """ """
    
        # Dimensions
        self.n = 2   
        self.m = 2   
        self.p = 2
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
        
        # Labels
        self.name = 'Holonomic Mobile Robot'
        self.state_label = ['x','y']
        self.input_label = ['vx', 'vy']
        self.output_label = ['x','y']
        
        # Units
        self.state_units = ['[m]','[m]']
        self.input_units = ['[m/sec]','[m/sec]']
        self.output_units = ['[m]','[m]']
        
        # State working range
        self.x_ub = np.array([ 10, 10])
        self.x_lb = np.array([-10,-10])
        
    #############################
    def f(self, x = np.zeros(3) , u = np.zeros(2) , t = 0 ):
        """ 
        Continuous time foward dynamics evaluation
        
        dx = f(x,u,t)
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        dx = np.zeros(self.n) # State derivative vector

        dx[0] = u[0]
        dx[1] = u[1] 
        
        return dx
    
    
    ###########################################################################
    # For graphical output
    ###########################################################################
    
    
    #############################
    def xut2q( self, x , u , t ):
        """ compute config q """
        
        q = x # kinematic model : state = config space
        
        return q
    
    ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 10
        
        domain  = [ ( -l , l ) ,
                    ( -l , l ) ,
                    ( -l , l ) ]#
            
                
        return domain
    
    ###########################################################################
    def forward_kinematic_lines(self, q ):
        """ 
        Compute points p = [x;y;z] positions given config q 
        ----------------------------------------------------
        - points of interest for ploting
        
        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines
        
        """
        
        lines_pts = [] # list of array (n_pts x 3) for each lines
        
        ###########################
        # Top line
        ###########################
            
        pts = np.zeros((4,3))
        
        d = 0.2
        
        pts[0,0] = q[0]+d
        pts[0,1] = q[1]+d
        pts[1,0] = q[0]+d
        pts[1,1] = q[1]-d
        pts[2,0] = q[0]-d
        pts[2,1] = q[1]-d
        pts[3,0] = q[0]-d
        pts[3,1] = q[1]+d
        
        lines_pts.append( pts )
                
        return lines_pts


##############################################################################
        
class HolonomicMobileRobotwithObstacles( HolonomicMobileRobot ):
    """ 
    
    dx   = u[0]
    dy   = u[1]
    
    """
    
    ############################
    def __init__(self):
        """ """
        # initialize standard params
        HolonomicMobileRobot.__init__(self)
        
        # Labels
        self.name = 'Holonomic Mobile Robot with Obstacles'

        # State working range
        self.x_ub = np.array([ 10, 10])
        self.x_lb = np.array([-10,-10])
        
        self.obstacles = [
                [ (2,2),(4,10)],
                [ (6,-8),(8,8)],
                [ (-8,-8),(-1,8)]
                ]
        
    #############################
    def isavalidstate(self , x ):
        """ check if x is in the state domain """
        ans = False
        for i in range(self.n):
            ans = ans or ( x[i] < self.x_lb[i] )
            ans = ans or ( x[i] > self.x_ub[i] )
        
        for obs in self.obstacles:
            on_obs = (( x[0] > obs[0][0]) and  
                      ( x[1] > obs[0][1]) and 
                      ( x[0] < obs[1][0]) and 
                      ( x[1] < obs[1][1]) )
                     
            ans = ans or on_obs
            
        return not(ans)
        
       
    ###########################################################################
    def forward_kinematic_lines(self, q ):
        """ 
        Compute points p = [x;y;z] positions given config q 
        ----------------------------------------------------
        - points of interest for ploting
        
        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines
        
        """
        
        lines_pts = [] # list of array (n_pts x 3) for each lines
        
        ###########################
        # Vehicule
        ###########################
            
        pts = np.zeros((4,3))
        
        d = 0.2
        
        pts[0,0] = q[0]+d
        pts[0,1] = q[1]+d
        pts[1,0] = q[0]+d
        pts[1,1] = q[1]-d
        pts[2,0] = q[0]-d
        pts[2,1] = q[1]-d
        pts[3,0] = q[0]-d
        pts[3,1] = q[1]+d
        
        lines_pts.append( pts )
        
        ###########################
        # obstacles
        ###########################
        
        for obs in self.obstacles:
            
            pts = np.zeros((5,3))
            
            pts[0,0] = obs[0][0]
            pts[0,1] = obs[0][1]
            
            pts[1,0] = obs[0][0]
            pts[1,1] = obs[1][1]
            
            pts[2,0] = obs[1][0]
            pts[2,1] = obs[1][1]
            
            pts[3,0] = obs[1][0]
            pts[3,1] = obs[0][1]
            
            pts[4,0] = obs[0][0]
            pts[4,1] = obs[0][1]
            
            lines_pts.append( pts )
            
                
        return lines_pts
        
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    sys = KinematicBicyleModel()
    
    sys.ubar = np.array([1,0.01])
    sys.plot_trajectory( np.array([0,0,0]) , 1000 )
    
    sys.animate_simulation( 10 )
        