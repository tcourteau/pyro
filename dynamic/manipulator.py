# -*- coding: utf-8 -*-
"""
Created on Wed May  8 08:47:05 2019

@author: alxgr
"""

###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic import mechanical
###############################################################################


###############################################################################
        
class Manipulator( mechanical.MechanicalSystem ):
    """ 
    Manipulator Robot 
    -------------------------------------------------------
    
    Dynamics:
    H(q) ddq + C(q,dq) dq + d(q,dq) + g(q) = B(q) u + J(q)^T f_ext
    
    Foward kinematic end-effector:
    r = foward_kinematic_effector(q)
    
    Foward differential kinematic effector:
    dr = J(q) dq
    
    Dimensions
    -------------------------------------------------------
    dof : number of degrees-of-freedom of the system
    n   : number of dynamics states (2 * dof)
    e   : number of effector dof
    m   : number of actuators inputs     
    -------------------------------------------------------
    
    Vectors
    -------------------------------------------------------
    q      :  dim = (dof, 1)   : position variables 
    dq     :  dim = (dof, 1)   : velocity variables     
    ddq    :  dim = (dof, 1)   : acceleration variables
    r      :  dim = (e, 1)     : end-effector positions
    dr     :  dim = (e, 1)     : end-effector velocities
    u      :  dim = (m, 1)     : force input variables
    f_ext  :  dim = (e, 1)     : end-effector external forces
    d      :  dim = (dof, 1)   : state-dependent dissipative forces
    g      :  dim = (dof, 1)   : state-dependent conservatives forces
    -------------------------------------------------------
    
    Matrix
    -------------------------------------------------------
    H(q)   :  dim = (dof, dof) : inertia matrix
    C(q)   :  dim = (dof, dof) : corriolis matrix
    B(q)   :  dim = (dof, m)   : actuator matrix
    J(q)   :  dim = (e , dof)  : end-effector Jacobian matrix
    -------------------------------------------------------
    
    """
    
    ############################
    def __init__(self, dof = 1 , m = 1 ):
        """ """
               
        # initialize standard params
        mechanical.MechanicalSystem.__init__(self, dof , m )
        
        # Name
        self.name = str(dof) + 'Joint Manipulator Robot'
        
    ###########################################################################
    # The following functions needs to be overloaded by child classes
    # to represent the dynamic of the system
    ###########################################################################
    
    # In Mother "Mechanical System" Class
    ###########################################################################
    # def H(self, q ):
    # def C(self, q , dq ):
    # def B(self, q ):
    # def g(self, q ):
    # def d(self, q , dq ):
    ###########################################################################
    
    # Specific to "Manipulator" Class
    
    ##############################
    def forward_kinematic_effector(self, q ):
        """ """
        
        r = np.zeros( self.e ) # Place holder
        
        return r
    
    ##############################
    def J(self, q ):
        """ """
        
        J = np.zeros( ( self.e  , self.dof ) ) # Place holder
        
        return J
    
    ##############################
    def f_ext(self, q , dq , t = 0 ):
        """ """
        
        f_ext = np.zeros( self.e ) # Default is zero vector
        
        return f_ext
    
    ###########################################################################
    # No need to overwrite the following functions for custom system
    ###########################################################################
    
    ##############################
    def forward_differential_kinematic_effector(self, q, dq ):
        """ """
        
        dr = np.dot( self.J(q) , dq )
        
        return dr
        
    
    ##############################
    def generalized_forces(self, q  , dq  , ddq , t = 0 ):
        """ Computed generalized forces given a trajectory """  
        
        H = self.H( q )
        C = self.C( q , dq )
        g = self.g( q )
        d = self.d( q , dq )
        
        f_ext = self.f_ext( q , dq , t )
        J     = self.J( q )
                
        # Generalized forces
        forces = ( np.dot( H , ddq ) + np.dot( C , dq ) + g + d 
                   - np.dot( J.T , f_ext ) )
        
        return forces
    
    
    ##############################
    def actuator_forces(self, q  , dq  , ddq , t = 0 ):
        """ Computed actuator forces given a trajectory (inverse dynamic) """  
        
        # Actuator Matrix
        B = self.B( q )
                
        # Generalized forces
        forces = self.generalized_forces( q , dq , ddq , t )
        
        # Actuator forces
        u = np.dot( np.linalg.inv( B ) , forces )
        
        return u
    
    
    ##############################
    def ddq(self, q , dq , u , t = 0 ):
        """ Computed accelerations given actuator forces (foward dynamic) """  
        
        H = self.H( q )
        C = self.C( q , dq )
        g = self.g( q )
        d = self.d( q , dq)
        B = self.B( q )
        
        f_ext = self.f_ext( q , dq , t )
        J     = self.J( q )
        
        ddq = np.dot( np.linalg.inv( H ) ,  ( + np.dot( B   , u )
                                              + np.dot( J.T , f_ext )
                                              - np.dot( C   , dq ) 
                                              - g 
                                              - d ) )
        return ddq
        



###############################################################################
# Two Link Manipulator
###############################################################################
        
class TwoLinkManipulator( Manipulator ):
    """ 

    """
    
    ############################
    def __init__(self):
        """ """
               
        # initialize standard params
        Manipulator.__init__(self, 2 , 2)
        
        # Name
        self.name = 'Two Link Manipulator'
        
        # params
        self.setparams()
                
            
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        self.l1  = 0.5
        self.l2  = 0.3
        self.lc1 = 0.2
        self.lc2 = 0.1
        
        self.m1 = 1
        self.I1 = 0
        self.m2 = 1
        self.I2 = 0
        
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
        c12 = np.cos( q[0] + q[1] )
        s12 = np.sin( q[0] + q[1] )
        
        return [c1,s1,c2,s2,c12,s12]
    
    
    ##############################
    def forward_kinematic_effector(self, q ):
        """ """
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        x = self.l1 * s1 + self.l2 * s12 # x
        y = self.l1 * c1 + self.l2 * c12 # y
        
        r = np.array([x,y])
        
        return r
    
    ##############################
    def J(self, q ):
        """ """
        
        J = np.zeros( ( self.e  , self.dof ) ) # Place holder
        
        J[0,0]
        
        return J
    
    ##############################
    def f_ext(self, q , dq , t = 0 ):
        """ """
        
        f_ext = np.zeros( self.e ) # Default is zero vector
        
        return f_ext
        
    
    ###########################################################################
    def H(self, q ):
        """ 
        Inertia matrix 
        ----------------------------------
        dim( H ) = ( dof , dof )
        
        such that --> Kinetic Energy = 0.5 * dq^T * H(q) * dq
        
        """  
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        H = np.zeros((2,2))
        
        H[0,0] = ( self.m1 * self.lc1**2 + self.I1 + self.m2 * ( self.l1**2 
                 + self.lc2**2 + 2 * self.l1 * self.lc2 * c2 ) + self.I2 )
        H[1,0] = ( self.m2 * self.lc2**2 + self.m2 * self.l1 * self.lc2 * c2 
                   + self.I2 )
        H[0,1] = H[1,0]
        H[1,1] = self.m2 * self.lc2 ** 2 + self.I2
        
        return H
    
    ###########################################################################
    def C(self, q , dq ):
        """ 
        Corriolis and Centrifugal Matrix 
        ------------------------------------
        dim( C ) = ( dof , dof )
        
        such that --> d H / dt =  C + C^T
        
        
        """ 
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        h = self.m2 * self.l1 * self.lc2 * s2
        
        C = np.zeros((2,2))
        
        C[0,0] = - h  * dq[1]
        C[1,0] =   h  * dq[0]
        C[0,1] = - h * ( dq[0] + dq[1] )
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
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        g1 = (self.m1 * self.lc1 + self.m2 * self.l1 ) * self.gravity
        g2 = self.m2 * self.lc2 * self.gravity
        
        G = np.zeros(2)
        
        G[0] = - g1 * s1 - g2 * s12
        G[1] = - g2 * s12

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
        
        pts      = np.zeros(( 2 , 3 ))
        pts[0,:] = np.array([-10,0,0])
        pts[1,:] = np.array([+10,0,0])
        
        lines_pts.append( pts )
        
        ###########################
        # pendulum kinematic
        ###########################
        
        pts      = np.zeros(( 3 , 3 ))
        pts[0,:] = np.array([0,0,0])
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        pts[1,0] = self.l1 * s1
        pts[1,1] = self.l1 * c1
        
        pts[2,0] = self.l1 * s1 + self.l2 * s12
        pts[2,1] = self.l1 * c1 + self.l2 * c12
        
        lines_pts.append( pts )
                
        return lines_pts
        