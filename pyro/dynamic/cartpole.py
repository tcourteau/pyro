# -*- coding: utf-8 -*-
"""
Created on Wed Oct 24 13:07:39 2018

@author: nvidia
@author: Thomas Courteau
"""
###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic import mechanical
from pyro.dynamic import system
###############################################################################


###############################################################################
        
class RotatingCartPole( mechanical.MechanicalSystem ):
    """ 

    """
    
    ############################
    def __init__(self):
        """ """
               
        # initialize standard params
        mechanical.MechanicalSystem.__init__(self, 2)
        
        # Name
        self.name = 'Rotating Cart Pole'
        
        # params
        self.setparams()
                
            
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        self.l1  = 1 
        self.l2  = 1
        
        self.m2  = 1
        
        self.I1 = 1.0
        self.I2 = 0.1
        
        self.gravity = 9.81
        
        self.d1 = 0.1
        self.d2 = 0.1
        
        
    ##############################
    def trig(self, q ):
        """ 
        Compute cos and sin usefull in other computation 
        ------------------------------------------------
        
        """
        
        c1  = np.cos( q[0] )
        s1  = np.sin( q[0] )
        c2  = np.cos( q[1] )
        s2  = np.sin( q[1] )
        
        return [c1,s1,c2,s2]
        
    
    ###########################################################################
    def H(self, q ):
        """ 
        Inertia matrix 
        ----------------------------------
        dim( H ) = ( dof , dof )
        
        such that --> Kinetic Energy = 0.5 * dq^T * H(q) * dq
        
        """  
        
        [c1,s1,c2,s2] = self.trig( q )
        
        H = np.zeros((2,2))
        
        H[0,0] = self.m2 * self.l1 ** 2 + self.I1
        H[1,0] = self.m2 * self.l1 * self.l2 * c2
        H[0,1] = H[1,0]
        H[1,1] = self.m2 * self.l2 ** 2 + self.I2
        
        return H
    
    ###########################################################################
    def C(self, q , dq ):
        """ 
        Corriolis and Centrifugal Matrix 
        ------------------------------------
        dim( C ) = ( dof , dof )
        
        such that --> d H / dt =  C + C^T
        
        
        """ 
        
        [c1,s1,c2,s2] = self.trig( q )
        
        C = np.zeros((2,2))
        
        C[0,0] = 0
        C[1,0] = 0
        C[0,1] = - self.m2 * self.l1 * self.l2 * s2 * dq[1]
        C[1,1] = 0

        
        return C
    
    ###########################################################################
    def B(self, q ):
        """ 
        Actuator Matrix  : dof x m
        """
        
        B = np.diag( np.ones( self.dof ) ) #  identity matrix
        
        return B
    
    ###########################################################################
    def g(self, q ):
        """ 
        Gravitationnal forces vector : dof x 1
        """
        
        [c1,s1,c2,s2] = self.trig( q )
        
        G = np.zeros(2)
        
        G[0] = 0
        G[1] = - self.m2 * self.gravity * self.l2 * s2
        
        return G
        
    ###########################################################################
    def d(self, q , dq ):
        """ 
        State-dependent dissipative forces : dof x 1
        """
        
        D = np.zeros((2,2))
        
        D[0,0] = self.d1
        D[1,0] = 0
        D[0,1] = 0
        D[1,1] = self.d2
        
        d = np.dot( D , dq )
        
        return d
        
    ###########################################################################
    # Graphical output
    ###########################################################################
    
    ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 2
        
        domain  = [ (-l,l) , (-l,l) , (-l,l) ]#  
                
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
        
        ###############################
        # ground line
        ###############################
        
        pts      = np.zeros(( 5 , 3 ))
        pts[0,:] = np.array([-1,-1,-1])
        pts[1,:] = np.array([-1,+1,-1])
        pts[2,:] = np.array([+1,+1,-1])
        pts[3,:] = np.array([+1,-1,-1])
        pts[4,:] = np.array([-1,-1,-1])
        
        lines_pts.append( pts )
        
        ###########################
        # pendulum kinematic
        ###########################
        
        pts      = np.zeros(( 4 , 3 ))
        
        [c1,s1,c2,s2] = self.trig( q )
        
        pts[0,0] = 0
        pts[0,1] = 0
        pts[0,2] = -1
        
        pts[1,0] = 0
        pts[1,1] = 0
        pts[1,2] = 0
        
        pts[2,0] = self.l1 * s1
        pts[2,1] = -(self.l1 * c1)
        pts[2,2] = 0
        
        pts[3,0] = self.l1 * s1   + self.l2 * s2 * c1
        pts[3,1] = - (self.l1 * c1   - self.l2 * s2 * s1)
        pts[3,2] = 0              + self.l2 * c2

        
        lines_pts.append( pts )
                
        return lines_pts
    
    
    
###############################################################################
        
class UnderActuatedRotatingCartPole( RotatingCartPole ):
    
    ############################
    def __init__(self, ):
        """ """
        
        # Degree of Freedom
        dof      = 2
        self.dof = dof 
        
        # Dimensions
        n = dof * 2 
        m = 1  
        p = dof * 2
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, n, m, p)
        
        # Name
        self.name = str(dof) + 'DoF Mechanical System'
        
        # Labels, bounds and units
        self.x_ub[0] = + np.pi * 2
        self.x_lb[0] = - np.pi * 2
        self.state_label[0] = 'Angle '+ str(1)
        self.state_units[0] = '[rad]'
        self.x_ub[1] = + np.pi * 2
        self.x_lb[1] = - np.pi * 2
        self.state_label[1] = 'Angle '+ str(2)
        self.state_units[1] = '[rad]'
            
        # joint velocity states
        self.x_ub[2] = + np.pi * 2
        self.x_lb[2] = - np.pi * 2
        self.state_label[2] = 'Velocity ' + str(1)
        self.state_units[2] = '[rad/sec]'
        self.x_ub[3] = + np.pi * 2
        self.x_lb[3] = - np.pi * 2
        self.state_label[3] = 'Velocity ' + str(2)
        self.state_units[3] = '[rad/sec]'
        
        #actuators
        self.u_ub[0] = + 5
        self.u_lb[0] = - 5
        self.input_label[0] = 'Torque ' + str(1)
        self.input_units[0] ='[Nm]'
        
        
        # Name
        self.name = 'Rotating Cart Pole'
        
        # params
        self.setparams()
    
    ###########################################################################
    def B(self, q ):
        """ 
        Actuator Matrix  : dof x m
        """
        
        B = np.zeros((2,1))
        
        B[0] = 1
        B[1] = 0
        
        return B


###############################################################################

class SelfBalanceRobot2D(mechanical.MechanicalSystem):
    """

    x[0] : Position of the cart
    x[1] : Angle of the pole

    u[0] : Torque of the wheel

    """

    def __init__(self, ):
        """ """

        # Degree of Freedom
        dof = 2
        self.dof = dof

        # Dimensions
        n = dof * 2
        m = 1
        p = dof * 2

        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, n, m, p)

        # Name
        # self.name = str(dof) + 'DoF Mechanical System'
        self.name = 'Self Balance Robot 2D'

        # Labels, bounds and units
        position_bound = 5     # [m]
        self.x_ub[0] = + position_bound
        self.x_lb[0] = - position_bound
        self.state_label[0] = 'Position'
        self.state_units[0] = '[m]'
        self.x_ub[1] = + np.pi
        self.x_lb[1] = - np.pi
        self.state_label[1] = 'Angle'
        self.state_units[1] = '[rad]'

        # joint velocity states
        velocity_bound = 10     # [m/s]
        self.x_ub[2] = + velocity_bound
        self.x_lb[2] = - velocity_bound
        self.state_label[2] = 'Velocity'
        self.state_units[2] = '[m/sec]'
        self.x_ub[3] = + np.pi * 2
        self.x_lb[3] = - np.pi * 2
        self.state_label[3] = 'Velocity'
        self.state_units[3] = '[rad/sec]'

        # actuators
        self.u_ub[0] = + 5
        self.u_lb[0] = - 5
        self.input_label[0] = 'Torque'
        self.input_units[0] = '[Nm]'

        # params
        self.setparams()

    #############################
    def setparams(self):
        """ Set model parameters here """

        self.lenght = 0.31

        # Masses of the system
        self.m_load = 0.164
        self.m_wheel = 0.053

        self.wheel_radius = 0.0425

        self.I1 = 0.0001  # Inertia of the body
        self.J = 0.0001   # Inertia of the wheels

        # Dissipative constantes
        self.viscosity = 0.025  # Viscosity friction

        self.gravity = 9.81

    ##############################
    def trig(self, q):
        """
        Compute cos and sin usefull in other computation
        ------------------------------------------------

        """

        c1 = np.cos(q[1])
        s1 = np.sin(q[1])

        return [c1, s1]

    ###########################################################################
    def H(self, q):
        """
        Inertia matrix
        ----------------------------------
        dim( H ) = ( dof , dof )

        such that --> Kinetic Energy = 0.5 * dq^T * H(q) * dq

        """

        [c1, s1] = self.trig(q)

        H = np.zeros((self.dof, self.dof))

        H[0, 0] = self.m_wheel + self.m_load + self.J / self.wheel_radius ** 2
        H[1, 0] = self.m_load * self.lenght * c1
        H[0, 1] = H[1, 0]
        H[1, 1] = self.m_load * self.lenght ** 2 + self.I1

        return H

    ###########################################################################
    def C(self, q, dq):
        """
        Corriolis and Centrifugal Matrix
        ------------------------------------
        dim( C ) = ( dof , dof )

        such that --> d H / dt =  C + C^T


        """

        [c1, s1] = self.trig(q)

        C = np.zeros((self.dof, self.dof))

        C[0, 0] = 0
        C[1, 0] = 0
        C[0, 1] = - self.m_load * self.lenght * s1 * dq[1]
        C[1, 1] = 0

        return C

    ###########################################################################
    def B(self, q):
        """
        Actuator Matrix  : dof x m
        """

        B = np.zeros((self.dof, self.m))

        B[0] = 1 / self.wheel_radius
        B[1] = -1

        return B

    ###########################################################################
    def g(self, q):
        """
        Gravitationnal forces vector : dof x 1
        """

        [c1, s1] = self.trig(q)

        G = np.zeros(self.dof)

        G[0] = 0
        G[1] = - self.m_load * self.gravity * self.lenght * s1

        return G

    ###########################################################################
    def d(self, q, dq):
        """
        State-dependent dissipative forces : dof x 1
        """

        D = np.zeros((self.dof, self.dof))

        D[0, 0] = self.viscosity / (self.wheel_radius**2)
        D[1, 0] = - self.viscosity / self.wheel_radius
        D[0, 1] = D[1, 0]
        D[1, 1] = self.viscosity

        d = np.dot(D, dq)

        return d

    def h(self, x, u, t):
        """
        TODO

        Output fonction y = h(x,u,t)

        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1

        OUTPUTS
        y  : output derivative vector o x 1

        """

        y = np.zeros(self.p) # Output vector

        y = x  # default output is all states

        return y


    ###########################################################################
    # Graphical output
    ###########################################################################

    ###########################################################################
    def forward_kinematic_domain(self, q):
        """
        """
        l = 0.5

        domain = [(-l, l), (-l, l), (-l, l)]  #

        return domain

    ###########################################################################
    def forward_kinematic_lines(self, q):
        """
        Compute points p = [x;y;z] positions given config q
        ----------------------------------------------------
        - points of interest for ploting

        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines

        """

        lines_pts = []  # list of array (n_pts x 3) for each lines

        ###########################
        # Top line
        ###########################

        pts = np.zeros((2, 3))

        pts[0, 0] = -10000
        pts[0, 1] = 0
        pts[1, 0] = 10000
        pts[1, 1] = 0

        lines_pts.append(pts)

        ###########################
        # Vehicule kinematic
        ###########################

        pts = np.zeros((2, 3))

        pts[0, 0] = q[0]
        pts[0, 1] = 0
        pts[1, 0] = q[0] + self.lenght * np.sin(q[1])
        pts[1, 1] = self.lenght * np.cos(q[1])

        lines_pts.append(pts)

        return lines_pts


###############################################################################

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
#    sys = RotatingCartPole()
#    sys = UnderActuatedRotatingCartPole()
    sys = SelfBalanceRobot2D()

    # Initiale state
    # x0[0] : Position of the cart
    # x0[1] : Angle of the rigid link
    # x0[2] : Speed of the cart
    # x0[3] : Angulare speed of the rigid link
    x0 = np.array([0, 0.12, 0, 0])

    # Initiale input
    # sys.ubar = np.array([0,0])
    
    #sys.show3(np.array([0.3,0.2]))
    
    sys.plot_trajectory( x0 , 0.64 ) #5, 50001, 'euler' )
#    sys.plot_phase_plane_trajectory( x0 , tf=0.64)
    sys.animate_simulation(0.1 , save=True)