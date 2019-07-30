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
        self.width  = 2.00
        self.a      = 2.00
        self.b      = 3.00
        self.lenght = self.a+self.b    
        self.lenght_tire = 0.40
        self.width_tire = 0.15
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

        dx[0] = u[1] * np.cos( x[2] )
        dx[1] = u[1] * np.sin( x[2] )
        dx[2] = u[1] * np.tan( u[0] ) * ( 1. / self.lenght) 
        
        return dx
    
    
    ###########################################################################
    # For graphical output
    ###########################################################################
    
    
    #############################
    def xut2q( self, x , u , t ):
        """ compute config q """
        
        q   = np.append(  x , u[0] ) # steering angle is part of the config
        
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
            
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = 1
        pts[1,0] = 10000
        pts[1,1] = 1
        
        lines_pts.append( pts )
        
        ###########################
        # bottom line
        ###########################
        
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = -1
        pts[1,0] = 10000
        pts[1,1] = -1
        
        lines_pts.append( pts )
        
        ###########################
        # Car
        ###########################
        
        """
        Here is how the car is drawn:
        
        |---------lenght--------|
        |-----a------|
        *                       *   -
            *               *       |
          d2    *   CG  *    d3     |
                    *              width
          d1    *       *    d4     |
           *                *       |
        *                       *   -
        """
        # Distance of the four corners of the car from the mass center
        d1 = np.sqrt(self.a**2+(self.width/2)**2)
        d3 = np.sqrt((self.lenght-self.a)**2+(self.width/2)**2)
        
        # Angles of the four lines of the car       
        theta1 = np.pi+np.arctan(self.width/2/self.a)+q[2]
        theta2 = np.pi-np.arctan(self.width/2/self.a)+q[2]
        theta3 = np.arctan(self.width/2/(self.lenght-self.a))+q[2]
        theta4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[2]
        
        # Build first line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d1,theta1,theta2,q[0],q[1])
        lines_pts.append( pts )
        
        # Build second line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d3,d3,theta3,theta4,q[0],q[1])
        px3 = pts[0,0] # Points used to center the wheels
        py3 = pts[0,1]
        px4 = pts[1,0]
        py4 = pts[1,1]
        lines_pts.append( pts )
        
        # Build third line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta2,theta3,q[0],q[1])
        lines_pts.append( pts )

        # Build third fourth of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta1,theta4,q[0],q[1])
        lines_pts.append( pts )
        
        ###########################
        # Wheels
        ###########################
        
        # Distance of the four corners of a tire from the center
        d  = np.sqrt((self.lenght_tire/2)**2+(self.width_tire/2)**2)
        
        # Angles of the four lines of a tire               
        steer1 = np.pi+np.arctan(self.width/2/self.a)+q[3]+q[2]
        steer2 = np.pi-np.arctan(self.width/2/self.a)+q[3]+q[2]
        steer3 = np.arctan(self.width/2/(self.lenght-self.a))+q[3]+q[2]
        steer4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[3]+q[2]
        
        # Build first line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px3,py3)
        lines_pts.append( pts )
        
        # Build second line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px3,py3)
        lines_pts.append( pts )

        # Build third line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px3,py3)
        lines_pts.append( pts )
        
        # Build fourth line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px3,py3)
        lines_pts.append( pts )
        
        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px4,py4)
        lines_pts.append( pts )
        
        # Build second line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px4,py4)
        lines_pts.append( pts )
        
        # Build third line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px4,py4)
        lines_pts.append( pts )

        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px4,py4)
        lines_pts.append( pts )
                
        return lines_pts
        
    ##########################################################################        
    def drawLine(self, d1, d2, angle1, angle2, x, y):
        
        x1 = x+d1*np.cos(angle1)
        y1 = y+d1*np.sin(angle1)
        x2 = x+d2*np.cos(angle2)
        y2 = y+d2*np.sin(angle2)
            
        return x1,y1,x2,y2

        
class KinematicBicyleModelwithObstacles( KinematicBicyleModel ):
    """ 
    
    """
    
    ############################
    def __init__(self):
        """ """
        # initialize standard params
        KinematicBicyleModel.__init__(self)
        
        # Labels
        self.name = 'Kinematic Bicyle Model with Obstacles'
        
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
        # Top line
        ###########################
            
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = 1
        pts[1,0] = 10000
        pts[1,1] = 1
        
        lines_pts.append( pts )
        
        ###########################
        # Bottom line
        ###########################
        
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = -1
        pts[1,0] = 10000
        pts[1,1] = -1
        
        lines_pts.append( pts )
        
        ###########################
        # Car
        ###########################
        
        """
        Here is how the car is drawn:
        
        |---------lenght--------|
        |-----a------|
        *                       *   -
            *               *       |
          d2    *   CG  *    d3     |
                    *              width
          d1    *       *    d4     |
           *                *       |
        *                       *   -
        """
        # Distance of the four corners of the car from the mass center
        d1 = np.sqrt(self.a**2+(self.width/2)**2)
        d3 = np.sqrt((self.lenght-self.a)**2+(self.width/2)**2)
        
        # Angles of the four lines of the car       
        theta1 = np.pi+np.arctan(self.width/2/self.a)+q[2]
        theta2 = np.pi-np.arctan(self.width/2/self.a)+q[2]
        theta3 = np.arctan(self.width/2/(self.lenght-self.a))+q[2]
        theta4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[2]
        
        # Build first line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d1,theta1,theta2,q[0],q[1])
        lines_pts.append( pts )
        
        # Build second line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d3,d3,theta3,theta4,q[0],q[1])
        px3 = pts[0,0] # Points used to center the wheels
        py3 = pts[0,1]
        px4 = pts[1,0]
        py4 = pts[1,1]
        lines_pts.append( pts )
        
        # Build third line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta2,theta3,q[0],q[1])
        lines_pts.append( pts )

        # Build third fourth of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta1,theta4,q[0],q[1])
        lines_pts.append( pts )
        
        ###########################
        # Wheels
        ###########################
        
        # Distance of the four corners of a tire from the center
        d  = np.sqrt((self.lenght_tire/2)**2+(self.width_tire/2)**2)
        
        # Angles of the four lines of a tire               
        steer1 = np.pi+np.arctan(self.width/2/self.a)+q[3]+q[2]
        steer2 = np.pi-np.arctan(self.width/2/self.a)+q[3]+q[2]
        steer3 = np.arctan(self.width/2/(self.lenght-self.a))+q[3]+q[2]
        steer4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[3]+q[2]
        
        # Build first line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px3,py3)
        lines_pts.append( pts )
        
        # Build second line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px3,py3)
        lines_pts.append( pts )

        # Build third line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px3,py3)
        lines_pts.append( pts )
        
        # Build fourth line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px3,py3)
        lines_pts.append( pts )
        
        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px4,py4)
        lines_pts.append( pts )
        
        # Build second line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px4,py4)
        lines_pts.append( pts )
        
        # Build third line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px4,py4)
        lines_pts.append( pts )

        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px4,py4)
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

###############################################################################
        
class LateralDynamicBicycleModelVI( system.ContinuousDynamicSystem ):
    """ 
    Equations of Motion
    -------------------------
    dv_y    = (F_yf*cos(delta)+F_yr)/m - v_x*dtheta         "lateral force sum (F_y = m*dv_y) gives lateral acceleration ddv_y"
    ddtheta = (a*F_yf*cos(delta)-b*F_yr)/Iz                 "Torque sum at mass center (T = I*ddtheta) gives the angular acceleration of the vehicle ddtheta"
    dtheta  = dtheta                                        "dtheta is already a state which is the yaw rate"
    dX      = v_x*cos(theta)-v_y*sin(theta)                 "To obtain cartesian position"
    dY      = v_x*sin(theta)+v_y*cos(theta)

    Where 
    F_yf is the lateral force applied perpendicularly with the front wheel (N)
    F_yr is the lateral force applied perpendicularly with the rear wheel (N)
    v_y  is the lateral velocity of the mass center (m/s)
    v_x  is the longitudinal velocity of the mass center (m/s)
    delta is the steering angle (rad)
    theta is the yaw angle of the vehicle (rad)
    m     is the mass of the vehicle (kg)
    Iz    is the inertia for a rotation along the z-axis (kg*m^2)
    a     is the distance from the front axle to the mass centre (m)
    b     is the distance from the rear axle to the mass centre (m)
    (X,Y) is the cartesian position of the vehicle
    """
    
    ############################
    def __init__(self):
        """ """
    
        # Dimensions
        self.n = 5   
        self.m = 2   
        self.p = 5
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
        
        # Labels
        self.name = 'Lateral Dynamic Bicyle Model'
        self.state_label = ['v_y','dtheta','theta','X','Y']
        self.input_label = ['delta', 'v_x']
        self.output_label = ['v_y','dtheta','theta','X','Y']
        
        # Units
        self.state_units = ['[m/s]','[rad/s]','[rad]','[m]','[m]']
        self.input_units = ['[rad]', '[m/sec]']
        self.output_units = ['[m/s]','[rad/s]','[rad]','[m]','[m]']

        
        # Model param (used for animation purposes only change the values)
        self.width  = 2.00
        self.a      = 2.00
        self.b      = 3.00
        self.lenght = self.a+self.b    
        self.lenght_tire = 0.40
        self.width_tire = 0.15
        
        self.g = 9.81
        self.mass = 2000
        self.Iz = 1.00/12.00*self.mass*((self.a+self.b)**2+self.width**2) 
        
        # Graphic output parameters 
        self.dynamic_domain  = True
    
        #############################
    def pieceWiseTireModel(self,x,u):
        
        #Tire-road friction coefficient
        self.mu = 0.9
        #Compute normal forces on the tires
        F_nf = self.mass*self.g*self.b/(self.b+self.a)
        F_nr = self.mass*self.g*self.a/(self.b+self.a)
        #Compute the max forces available
        max_F_f = F_nf*self.mu
        max_F_r = F_nr*self.mu
        #Compute the lateral "slip-slope"
        self.max_alpha_stat = 0.12
        slip_ratio_f = max_F_f/(self.max_alpha_stat)
        slip_ratio_r = max_F_r/(self.max_alpha_stat)
        if (u[1] == 0):
            slip_f = 0
            slip_r = 0
            F_yf   = 0
            F_yr   = 0
        else:
            slip_f = u[0]-np.arctan((x[0]+self.a*x[1])/u[1])
            slip_r = np.arctan((self.b*x[1]-x[0])/u[1])
            
        if (slip_f<-0.12):
            F_yf = -max_F_f
        elif (slip_f > 0.12):
            F_yf = max_F_f
        else:
            F_yf = slip_ratio_f*slip_f
            
        if (slip_r<-0.12):
            F_yr = -max_F_r
        elif (slip_r > 0.12):
            F_yr = max_F_r
        else:
            F_yr = slip_ratio_r*slip_r

        return F_yf,F_yr   
        
    #############################
        
    def f(self, x = np.zeros(5) , u = np.zeros(2) , t = 0 ):
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
        F_yf,F_yr = self.pieceWiseTireModel(x,u)
        
        dx[0] = (F_yf*np.cos(u[0])+F_yr)/self.mass-u[1]*x[1]
        dx[1] = (self.a*F_yf*np.cos(u[0])-self.b*F_yr)/self.Iz
        dx[2] = x[1]
        dx[3] = u[1]*np.cos(x[2])-x[0]*np.sin(x[2])
        dx[4] = u[1]*np.sin(x[2])-x[0]*np.cos(x[2])
        
        return dx

    ###########################################################################
    # For graphical output
    ###########################################################################
    
    
    #############################
    def xut2q( self, x , u , t ):
        """ compute config q """
        q   = np.append(  x , u[0] ) # steering angle is part of the config
        
        return q
    
    ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 10
        
        x = q[3]
        y = q[4]
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
            
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = 1
        pts[1,0] = 10000
        pts[1,1] = 1
        
        lines_pts.append( pts )
        
        ###########################
        # bottom line
        ###########################
        
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = -1
        pts[1,0] = 10000
        pts[1,1] = -1
        
        lines_pts.append( pts )
        
        ###########################
        # Car
        ###########################
        
        """
        Here is how the car is drawn:
        
        |---------lenght--------|
        |-----a------|
        *                       *   -
            *               *       |
          d2    *   CG  *    d3     |
                    *              width
          d1    *       *    d4     |
           *                *       |
        *                       *   -
        """
        # Distance of the four corners of the car from the mass center
        d1 = np.sqrt(self.a**2+(self.width/2)**2)
        d3 = np.sqrt((self.lenght-self.a)**2+(self.width/2)**2)
        
        # Angles of the four lines of the car       
        theta1 = np.pi+np.arctan(self.width/2/self.a)+q[2]
        theta2 = np.pi-np.arctan(self.width/2/self.a)+q[2]
        theta3 = np.arctan(self.width/2/(self.lenght-self.a))+q[2]
        theta4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[2]
        
        # Build first line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d1,theta1,theta2,q[3],q[4])
        lines_pts.append( pts )
        
        # Build second line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d3,d3,theta3,theta4,q[3],q[4])
        px3 = pts[0,0] # Points used to center the wheels
        py3 = pts[0,1]
        px4 = pts[1,0]
        py4 = pts[1,1]
        lines_pts.append( pts )
        
        # Build third line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta2,theta3,q[3],q[4])
        lines_pts.append( pts )

        # Build third fourth of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta1,theta4,q[3],q[4])
        lines_pts.append( pts )
        
        ###########################
        # Wheels
        ###########################
        
        # Distance of the four corners of a tire from the center
        d  = np.sqrt((self.lenght_tire/2)**2+(self.width_tire/2)**2)
        
        # Angles of the four lines of a tire               
        steer1 = np.pi+np.arctan(self.width/2/self.a)+q[5]+q[2]
        steer2 = np.pi-np.arctan(self.width/2/self.a)+q[5]+q[2]
        steer3 = np.arctan(self.width/2/(self.lenght-self.a))+q[5]+q[2]
        steer4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[5]+q[2]
        
        # Build first line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px3,py3)
        lines_pts.append( pts )
        
        # Build second line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px3,py3)
        lines_pts.append( pts )


        # Build third line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px3,py3)
        lines_pts.append( pts )
        
        # Build fourth line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px3,py3)
        lines_pts.append( pts )
        
        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px4,py4)
        lines_pts.append( pts )
        
        # Build second line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px4,py4)
        lines_pts.append( pts )
        
        # Build third line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px4,py4)
        lines_pts.append( pts )

        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px4,py4)
        lines_pts.append( pts )
                
        return lines_pts
        
    ##########################################################################        
    def drawLine(self, d1, d2, angle1, angle2, x, y):
        
        x1 = x+d1*np.cos(angle1)
        y1 = y+d1*np.sin(angle1)
        x2 = x+d2*np.cos(angle2)
        y2 = y+d2*np.sin(angle2)
            
        return x1,y1,x2,y2
##############################################################################
        
class LateralDynamicBicycleModelFI( LateralDynamicBicycleModelVI ):
    
    """ 
    Equations of Motion
    -------------------------
<<<<<<< HEAD
    dx   = V cos ( phi )
    dy   = V sin ( phi )
    dphi = V/l tan ( beta )
=======
    STATES_DERIVATIVE
    dv_x    = (F_xf*cos(delta)-F_yf*sin(delta)+F_xr)/m-v_y*dtheta   "longitudinal force sum (F_x = m*dv_x) gives longitudinal acceleration dv_x"
    dv_y    = (F_yf*cos(delta)+F_xf*sin(delta)+F_yr)/m - v_x*dtheta "lateral force sum (F_y = m*dv_y) gives lateral acceleration dv_y"
    ddtheta = (a*(F_yf*cos(delta)+F_xf*sin(delta))-b*F_yr)/Iz       "Torque sum at mass center (T = I*ddtheta) gives the angular acceleration of the vehicle ddtheta"
    dtheta  = dtheta                                                "dtheta is already a state which is the yaw rate"
    dX      = v_x*cos(theta)-v_y*sin(theta)                         "To obtain cartesian position"
    dY      = v_x*sin(theta)+v_y*cos(theta)
    
    INPUTS 
    delta is the steering angle (rad)
    F_xf is the longitudinal force given by the front propulsion (N)
    F_xr is the longitudinal force given by the rear propulsion (N)
    
    
    Where 
    F_yf is the lateral force applied perpendicularly with the front wheel (N)
    F_yr is the lateral force applied perpendicularly with the rear wheel (N)
    v_y  is the lateral velocity of the mass center (m/s)
    v_x  is the longitudinal velocity of the mass center (m/s)
    delta is the steering angle (rad)
    theta is the yaw angle of the vehicle (rad)
    m     is the mass of the vehicle (kg)
    Iz    is the inertia for a rotation along the z-axis (kg*m^2)
    a     is the distance from the front axle to the mass centre (m)
    b     is the distance from the rear axle to the mass centre (m)
    (X,Y) is the cartesian position of the vehicle
>>>>>>> Will
    """
    
    ############################
    def __init__(self):
        """ """
    
        # Dimensions
        self.n = 6   
        self.m = 3   
        self.p = 6
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
        
        # Labels
        self.name = 'Full Dynamic Bicyle Model without slip'
        self.state_label = ['v_x','v_y','dtheta','theta','X','Y']
        self.input_label = ['delta', 'F_xf','F_xr']
        self.output_label = ['v_x','v_y','dtheta','theta','X','Y']
        
        # Units
        self.state_units = ['[m/s]','[m/s]','[rad/s]','[rad]','[m]','[m]']
        self.input_units = ['[rad]', '[N]','[N]']
        self.output_units = ['[m/s]','[m/s]','[rad/s]','[rad]','[m]','[m]']

        
        # Model param (used for animation purposes only change the values)
        self.width  = 2.00
        self.a      = 2.00
        self.b      = 3.00
        self.lenght = self.a+self.b    
        self.lenght_tire = 0.40
        self.width_tire = 0.15
        
        self.g = 9.81
        self.mass = 1000
        self.Iz = 1.00/12.00*self.mass*((self.a+self.b)**2+self.width**2) 
        
        # Graphic output parameters 
        self.dynamic_domain  = True
        
    #############################
    def pieceWiseTireModel(self,x,u):
        
        #Tire-road friction coefficient
        self.mu = 0.9
        F_nf = self.mass*self.g*self.b/(self.b+self.a)
        F_nr = self.mass*self.g*self.a/(self.b+self.a)
        #Compute the max forces available
        max_F_f = F_nf*self.mu
        max_F_r = F_nr*self.mu
        #Compute the lateral "slip-slope"
        self.max_alpha_stat = 0.12
        slip_ratio_f = max_F_f/(self.max_alpha_stat)
        slip_ratio_r = max_F_r/(self.max_alpha_stat)
        if x[0] == 0:
            slip_f = 0
            slip_r = 0
        elif(x[0]>0):
            slip_f = np.arctan((x[1]+self.a*x[2])/x[0])-u[0]
            slip_r = np.arctan((x[1]-self.b*x[2])/x[0])
        else:
            slip_f = -(np.arctan((x[1]+self.a*x[2])/x[0])+u[0])
            slip_r = -np.arctan((x[1]-self.b*x[2])/x[0])            
        if (slip_f<-0.12):
            F_yf = max_F_f
        elif (slip_f > 0.12):
            F_yf = -max_F_f
        else:
            F_yf = -slip_ratio_f*slip_f
            
        if (slip_r<-0.12):
            F_yr = max_F_r
        elif (slip_r > 0.12):
            F_yr = -max_F_r
        else:
            F_yr = -slip_ratio_r*slip_r
        
        return F_yf,F_yr         
    #############################
        
    def f(self, x = np.zeros(6) , u = np.zeros(3) , t = 0 ):
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
        F_yf,F_yr = self.pieceWiseTireModel(x,u)
        
        dx[0] = (u[1]*np.cos(u[0])-F_yf*np.sin(u[0])+u[2])/self.mass-x[1]*x[2]
        dx[1] = (F_yf*np.cos(u[0])+u[1]*np.sin(u[0])+F_yr)/self.mass-x[0]*x[2]
        dx[2] = (self.a*(F_yf*np.cos(u[0])+u[1]*np.sin(u[0]))-self.b*F_yr)/self.Iz
        dx[3] = x[2]
        dx[4] = x[0]*np.cos(x[3])-x[1]*np.sin(x[3])
        dx[5] = x[0]*np.sin(x[3])+x[1]*np.cos(x[3])
        
        return dx
    
        ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 10
        
        x = q[4]
        y = q[5]
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
            
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = 1
        pts[1,0] = 10000
        pts[1,1] = 1
        
        lines_pts.append( pts )
        
        ###########################
        # bottom line
        ###########################
        
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = -1
        pts[1,0] = 10000
        pts[1,1] = -1
        
        lines_pts.append( pts )
        
        ###########################
        # Car
        ###########################
        
        """
        Here is how the car is drawn:
        
        |---------lenght--------|
        |-----a------|
        *                       *   -
            *               *       |
          d2    *   CG  *    d3     |
                    *              width
          d1    *       *    d4     |
           *                *       |
        *                       *   -
        """
        # Distance of the four corners of the car from the mass center
        d1 = np.sqrt(self.a**2+(self.width/2)**2)
        d3 = np.sqrt((self.lenght-self.a)**2+(self.width/2)**2)
        
        # Angles of the four lines of the car       
        theta1 = np.pi+np.arctan(self.width/2/self.a)+q[3]
        theta2 = np.pi-np.arctan(self.width/2/self.a)+q[3]
        theta3 = np.arctan(self.width/2/(self.lenght-self.a))+q[3]
        theta4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[3]
        
        # Build first line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d1,theta1,theta2,q[4],q[5])
        lines_pts.append( pts )
        
        # Build second line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d3,d3,theta3,theta4,q[4],q[5])
        px3 = pts[0,0] # Points used to center the wheels
        py3 = pts[0,1]
        px4 = pts[1,0]
        py4 = pts[1,1]
        lines_pts.append( pts )
        
        # Build third line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta2,theta3,q[4],q[5])
        lines_pts.append( pts )

        # Build third fourth of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta1,theta4,q[4],q[5])
        lines_pts.append( pts )
        
        ###########################
        # Wheels
        ###########################
        
        # Distance of the four corners of a tire from the center
        d  = np.sqrt((self.lenght_tire/2)**2+(self.width_tire/2)**2)
        
        # Angles of the four lines of a tire               
        steer1 = np.pi+np.arctan(self.width/2/self.a)+q[6]+q[3]
        steer2 = np.pi-np.arctan(self.width/2/self.a)+q[6]+q[3]
        steer3 = np.arctan(self.width/2/(self.lenght-self.a))+q[6]+q[3]
        steer4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[6]+q[3]
        
        # Build first line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px3,py3)
        lines_pts.append( pts )
        
        # Build second line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px3,py3)
        lines_pts.append( pts )

        # Build third line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px3,py3)
        lines_pts.append( pts )
        
        # Build fourth line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px3,py3)
        lines_pts.append( pts )
        
        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px4,py4)
        lines_pts.append( pts )
        
        # Build second line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px4,py4)
        lines_pts.append( pts )
        
        # Build third line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px4,py4)
        lines_pts.append( pts )

        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px4,py4)
        lines_pts.append( pts )
                
        return lines_pts
    
##############################################################################
        
class FullDynamicBicycleModelTI( LateralDynamicBicycleModelVI ):
    
    """ 
    Equations of Motion
    -------------------------
    STATES_DERIVATIVE
    dv_x    = (F_xf*cos(delta)-F_yf*sin(delta)+F_xr)/m-v_y*dtheta   "longitudinal force sum (F_x = m*dv_x) gives longitudinal acceleration dv_x"
    dv_y    = (F_yf*cos(delta)+F_xf*sin(delta)+F_yr)/m - v_x*dtheta "lateral force sum (F_y = m*dv_y) gives lateral acceleration dv_y"
    ddtheta = (a*(F_yf*cos(delta)+F_xf*sin(delta))-b*F_yr)/Iz       "Torque sum at mass center (T = I*ddtheta) gives the angular acceleration of the vehicle ddtheta"
    dtheta  = dtheta                                                "dtheta is already a state which is the yaw rate"
    dX      = v_x*cos(theta)-v_y*sin(theta)                         "To obtain cartesian position"
    dY      = v_x*sin(theta)+v_y*cos(theta)
    domega_f = (T_F-F_xf*R)/I_propf
    domega_r = (T_r-F_xr*R)/I_propr
    
    ***** DEPENDS DU MODELE DE TIRE COMPLET EX: S=omega/v_x et F_x,F_y = f(F_z,alpha,S et mu) *****
    
    INPUTS 
    delta is the steering angle (rad)
    T_f is the front wheel torque (N)
    T_r is the rear wheel torque (N)
    
    
    Where 
    F_yf is the lateral force applied perpendicularly with the front wheel (N)
    F_yr is the lateral force applied perpendicularly with the rear wheel (N)
    v_y  is the lateral velocity of the mass center (m/s)
    v_x  is the longitudinal velocity of the mass center (m/s)
    delta is the steering angle (rad)
    theta is the yaw angle of the vehicle (rad)
    m     is the mass of the vehicle (kg)
    Iz    is the inertia for a rotation along the z-axis (kg*m^2)
    a     is the distance from the front axle to the mass centre (m)
    b     is the distance from the rear axle to the mass centre (m)
    (X,Y) is the cartesian position of the vehicle
    """
    
    ############################
    def __init__(self):
        """ """
    
        # Dimensions
        self.n = 8   
        self.m = 3   
        self.p = 8
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
        
        # Labels
        self.name = 'Full Dynamic Bicyle Model with slip'
        self.state_label = ['v_x','v_y','dtheta','theta','X','Y','omega_f','omega_r']
        self.input_label = ['delta', 'T_f','T_r']
        self.output_label = ['v_x','v_y','dtheta','theta','X','Y','omega_f','omega_r']
        
        # Units
        self.state_units = ['[m/s]','[m/s]','[rad/s]','[rad]','[m]','[m]','[rad/s]','[rad/s]']
        self.input_units = ['[rad]', '[Nm]','[Nm]']
        self.output_units = ['[m/s]','[m/s]','[rad/s]','[rad]','[m]','[m]','[rad/s]','[rad/s]']

        
        # Model param (used for animation purposes only change the values)
        self.width  = 2.00
        self.a      = 2.00
        self.b      = 3.00
        self.lenght = self.a+self.b    
        self.lenght_tire = 0.40
        self.width_tire = 0.15
        
        self.g = 9.81
        self.mass = 1000
        self.Iz = 1.00/12.00*self.mass*((self.a+self.b)**2+self.width**2) 
        self.Iprop_f = 20
        self.Iprop_r = 20
        self.r = 0.3
        
        # Graphic output parameters 
        self.dynamic_domain  = True
        
    #############################
    def combinedSlipTireModel(self,x,u):
        
        #Tire-road friction coefficient
        self.mu = 0.9
        F_nf = self.mass*self.g*self.b/(self.b+self.a)
        F_nr = self.mass*self.g*self.a/(self.b+self.a)
        #Compute the max forces available
        max_F_f = F_nf*self.mu
        max_F_r = F_nr*self.mu
        #Compute the lateral "slip-slope"
        self.max_alpha_stat = 0.12
        slip_ratio_f = max_F_f/(self.max_alpha_stat)
        slip_ratio_r = max_F_r/(self.max_alpha_stat)
        # Simulation of road-tire behavior according to slip-ratios
        self.max_slip_stat = 0.03
        slip_slope_f = max_F_f/self.max_slip_stat
        slip_slope_r = max_F_r/self.max_slip_stat
        # Compute slip ratios and longitudinal forces for both tires (depend on Vx (x[0]), omega_f (x[6]), omega_r (x[7]))
        if(x[0] == 0 or x[6] == 0):
            long_slip_f = 0
            F_xf = 0
        elif(u[1] == 0):
            long_slip_f = 0
            F_xf = 0
        else:
            long_slip_f = (x[6]*self.r-x[0])/np.maximum(np.absolute(x[0]),np.absolute(x[6]*self.r))  
            if (long_slip_f<-0.03):
                F_xf = -max_F_f
            elif(long_slip_f>0.03):
                F_xf = max_F_f
            else:
                F_xf = slip_slope_f*long_slip_f
        if(x[0] == 0 and x[7] == 0):
            F_xr = 0
            long_slip_r = 0
        elif(u[2] == 0):
            long_slip_r = 0
            F_xr = 0
        else:
            long_slip_r = (x[7]*self.r-x[0])/max(x[0],x[7]*self.r)
            if (long_slip_r<-0.03):
                F_xr = -max_F_r
            elif(long_slip_r>0.03):
                F_xr = max_F_r
            else:
                F_xr = slip_slope_r*long_slip_r
        # Compute slip angles and lateral forces for both tires (depends on Vx (x[0]),Vy (x[1]) and delta (u[0]))
        if x[0] == 0:
            slip_f = 0
            slip_r = 0
        elif(x[0]>0):
            slip_f = np.arctan((x[1]+self.a*x[2])/x[0])-u[0]
            slip_r = np.arctan((x[1]-self.b*x[2])/x[0])
        else:
            slip_f = -(np.arctan((x[1]+self.a*x[2])/x[0])-u[0])
            slip_r = -np.arctan((x[1]-self.b*x[2])/x[0])
        if (slip_f<-0.12):
            F_yf = max_F_f
        elif (slip_f > 0.12):
            F_yf = -max_F_f
        else:
            F_yf = -slip_ratio_f*slip_f
            
        if (slip_r<-0.1):
            F_yr = max_F_r
        elif (slip_r > 0.1):
            F_yr = -max_F_r
        else:
            F_yr = -slip_ratio_r*slip_r
        
        return F_yf,F_yr,F_xf,F_xr  
    #############################
        
    def f(self, x = np.zeros(8) , u = np.zeros(3) , t = 0 ):
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
        F_yf,F_yr,F_xf,F_xr = self.combinedSlipTireModel(x,u)
        
        dx[0] = (F_xf*np.cos(u[0])-F_yf*np.sin(u[0])+F_xr)/self.mass-x[1]*x[2]
        dx[1] = (F_yf*np.cos(u[0])+F_xf*np.sin(u[0])+F_yr)/self.mass-x[0]*x[2]
        dx[2] = (self.a*(F_yf*np.cos(u[0])+F_xf*np.sin(u[0]))-self.b*F_yr)/self.Iz
        dx[3] = x[2]
        dx[4] = x[0]*np.cos(x[3])-x[1]*np.sin(x[3])
        dx[5] = x[0]*np.sin(x[3])+x[1]*np.cos(x[3])
        dx[6] = (u[1]-self.r*F_xf)/self.Iprop_f
        dx[7] = (u[2]-self.r*F_xr)/self.Iprop_r
        
        return dx
    
        ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 10
        
        x = q[4]
        y = q[5]
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
            
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = 1
        pts[1,0] = 10000
        pts[1,1] = 1
        
        lines_pts.append( pts )
        
        ###########################
        # bottom line
        ###########################
        
        pts = np.zeros((2,10))
        
        pts[0,0] = -10000
        pts[0,1] = -1
        pts[1,0] = 10000
        pts[1,1] = -1
        
        lines_pts.append( pts )
        
        ###########################
        # Car
        ###########################
        
        """
        Here is how the car is drawn:
        
        |---------lenght--------|
        |-----a------|
        *                       *   -
            *               *       |
          d2    *   CG  *    d3     |
                    *              width
          d1    *       *    d4     |
           *                *       |
        *                       *   -
        """
        # Distance of the four corners of the car from the mass center
        d1 = np.sqrt(self.a**2+(self.width/2)**2)
        d3 = np.sqrt((self.lenght-self.a)**2+(self.width/2)**2)
        
        # Angles of the four lines of the car       
        theta1 = np.pi+np.arctan(self.width/2/self.a)+q[3]
        theta2 = np.pi-np.arctan(self.width/2/self.a)+q[3]
        theta3 = np.arctan(self.width/2/(self.lenght-self.a))+q[3]
        theta4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[3]
        
        # Build first line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d1,theta1,theta2,q[4],q[5])
        lines_pts.append( pts )
        
        # Build second line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d3,d3,theta3,theta4,q[4],q[5])
        px3 = pts[0,0] # Points used to center the wheels
        py3 = pts[0,1]
        px4 = pts[1,0]
        py4 = pts[1,1]
        lines_pts.append( pts )
        
        # Build third line of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta2,theta3,q[4],q[5])
        lines_pts.append( pts )

        # Build third fourth of the car 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d1,d3,theta1,theta4,q[4],q[5])
        lines_pts.append( pts )
        
        ###########################
        # Wheels
        ###########################
        
        # Distance of the four corners of a tire from the center
        d  = np.sqrt((self.lenght_tire/2)**2+(self.width_tire/2)**2)
        
        # Angles of the four lines of a tire               
        steer1 = np.pi+np.arctan(self.width/2/self.a)+q[8]+q[3]
        steer2 = np.pi-np.arctan(self.width/2/self.a)+q[8]+q[3]
        steer3 = np.arctan(self.width/2/(self.lenght-self.a))+q[8]+q[3]
        steer4 = 2*np.pi-np.arctan(self.width/2/(self.lenght-self.a))+q[8]+q[3]
        
        # Build first line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px3,py3)
        lines_pts.append( pts )
        
        # Build second line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px3,py3)
        lines_pts.append( pts )

        # Build third line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px3,py3)
        lines_pts.append( pts )
        
        # Build fourth line of the left tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px3,py3)
        lines_pts.append( pts )
        
        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer2,px4,py4)
        lines_pts.append( pts )
        
        # Build second line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer3,steer4,px4,py4)
        lines_pts.append( pts )
        
        # Build third line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer2,steer3,px4,py4)
        lines_pts.append( pts )

        # Build first line of the right tire 
        pts = np.zeros((2,10))
        pts[0,0],pts[0,1],pts[1,0],pts[1,1] = self.drawLine(d,d,steer1,steer4,px4,py4)
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
    
    sys.animate_simulation( 100 )
        