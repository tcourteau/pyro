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
    def generalized_forces(self, q  , dq  , ddq , t = 0 ):
        """ Computed generalized forces given a trajectory """  
        
        H = self.H( q )
        C = self.C( q , dq )
        g = self.g( q )
        d = self.d( q , dq )
                
        # Generalized forces
        forces = np.dot( H , ddq ) + np.dot( C , dq ) + g + d
        
        return forces
    
    ##############################
    def actuator_forces(self, q  , dq  , ddq , t = 0 ):
        """ Computed actuator forces given a trajectory (inverse dynamic) """  
        
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
        g = self.g( q  )
        d = self.d( q , dq)
        B = self.B( q )
        
        ddq = np.dot( np.linalg.inv( H ) ,  ( np.dot( B , u )  
                                            - np.dot( C , dq ) - g - d ) )
        
        return ddq
    
    
    ###########################################################################
    def f(self, x , u , t = 0 ):
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
        
        # from state vector (x) to angle and speeds (q,dq)
        [ q , dq ] = self.x2q( x )       
        
        # compute joint acceleration 
        ddq = self.ddq( q , dq , u , t ) 
        
        # from angle and speeds diff (dq,ddq) to state vector diff (dx)
        dx = self.q2x( dq , ddq )        
        
        return dx
        
        
        