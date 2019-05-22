# -*- coding: utf-8 -*-
"""
Created on Tue Oct 23 20:45:37 2018

@author: Alexandre
"""


###############################################################################
import numpy as np
##############################################################################
from pyro.dynamic import system
###############################################################################


###############################################################################
        
class KinematicBicyleModel( system.ContinuousDynamicSystem ):
    """ 
    Equations of Motion
    -------------------------
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


##############################################################################

class QuadcopterWithMass( system.ContinuousDynamicSystem ):
    """
    Equations of Motion
    -------------------------
    dx   = xd
    dxd  = (dot(p , f R n_z) - mq l dot(pd , pd))p / (mQ + mL) - g n_z
    dp   = pd
    dpd  = cross() / mQl - dot(pd , pd)p
    dR   = R O
    dO   = II^-1 (M - cross(O , IIO))
    """

    ############################
    def __init__(self):
        """ """

        # Dimensions
        self.n = 24
        self.m = 4
        self.p = 24

        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)

        # Labels
        self.name = 'Kinematic Quadcopter with mass Model'
        self.state_label = ['xLx','xLy','xLz','xdLx','xdLy','xdLz','px','py','pz','pdx','pdy','pdz','R1','R2','R3','R4','R5','R6','R7','R8','R9','Omegax','Omegay','Omegaz']
        self.input_label = ['f', 'Mx', 'My', 'Mz']
        self.output_label = ['xLx','xLy','xLz','xdLx','xdLy','xdLz','px','py','pz','pdx','pdy','pdz','R1','R2','R3','R4','R5','R6','R7','R8','R9','Omegax','Omegay','Omegaz']

        # Units
        self.state_units = ['[m]','[m]','[m]','[m/s]','[m/s]','[m/s]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[rad/s]','[rad/s]','[rad/s]']
        self.input_units = ['[N]', '[Nm]', '[Nm]', '[Nm]']
        self.output_units = ['[m]','[m]','[m]','[m/s]','[m/s]','[m/s]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[]','[rad/s]','[rad/s]','[rad/s]']

        # State working range
        x_range = np.ones(3) * 10
        xd_range = np.ones(3) * 100
        p_range = np.ones(3)
        pd_range = np.ones(3) * 5
        R_range = np.ones(9)
        Omega_range = np.ones(3) * 10
        self.x_ub = np.concatenate((x_range,xd_range,p_range,pd_range,R_range,Omega_range))
        self.x_lb = np.concatenate((-x_range,-xd_range,-p_range,-pd_range,-R_range,-Omega_range))

        # Model param
        self.lenght = 1
        self.mQ = 1
        self.mL = 0.5
        self.inertia = np.array([[0.011,0,0] , [0,0.015,0] , [0,0,0.021]])
        self.gravity = 9.81 # [m/s^2]
        self.n_z = np.array([0,0,1])

        # Graphic output parameters
        self.dynamic_domain = True

    #############################
    def f(self, x = np.zeros(24) , u = np.zeros(4) , t = 0 ):
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

        xdL = x[3:6]
        p = x[6:9]
        pd = x[9:12]
        R = x[12:21].reshape(3,3)
        Omega = x[21:24]

        f = u[0]
        M = u[1:4]

        dx = np.zeros(self.n) # State derivative vector

        dx[0:3] = xdL

        dx[3:6] = (np.dot(p , f * np.dot(R , self.n_z)) - self.mQ * self.lenght * np.dot(pd,pd))*p / (self.mQ + self.mL) - self.gravity * self.n_z
        dx[6:9] = pd
        dx[9:12] = np.cross(p , np.cross(p , f*np.dot(R , self.n_z))) / (self.mQ * self.lenght) - np.dot(pd , pd)*p

        Omega_skew = np.zeros(9).reshape(3,3)
        Omega_skew[0,1] = -Omega[2]
        Omega_skew[0,2] = Omega[1]
        Omega_skew[1,0] = Omega[2]
        Omega_skew[1,2] = -Omega[0]
        Omega_skew[2,0] = -Omega[1]
        Omega_skew[2,1] = Omega[0]
        Rd = np.dot(R , Omega_skew)
        dx[12:21] = Rd.flatten()

        dx[21:24] = np.dot(np.linalg.inv(self.inertia) , (M - np.cross(Omega , np.dot(self.inertia , Omega))))

        return dx
        

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """

    ## Quad exemple
    quad = QuadcopterWithMass()

    # Initiales conditions
    xL = np.array([0,0,5]) # [m]
    xdL = np.array([0,0,0]) # [m/s]
    p = np.array([0,0,-1])
    pd = np.array([0,0,0])
    R = np.identity(3)
    Omega = np.array([0,0,0])
    ci = np.concatenate((xL,xdL,p,pd,R.flatten(),Omega))

    f_ini = (quad.mQ + quad.mL) * quad.gravity

    quad.ubar = np.array([f_ini,0,0,0])
    quad.plot_trajectory( ci , 1000 )
