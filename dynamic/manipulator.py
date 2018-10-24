# -*- coding: utf-8 -*-
"""
Created on Tue Oct 23 20:45:37 2018

@author: Alexandre
"""

import numpy as np

from AlexRobotics.core import system


##############################################################################
        
class Manipulator( system.ContinuousDynamicSystem ):
    """ 
    Mechanical system with Equation of Motion in the form of
    -------------------------------------------------------
    H(q) ddq + C(q,dq) dq + g(q) = B(q) tau
    
    """
    
    ############################
    def __init__(self, dof = 1 ):
        """ """
        
        # Degree of Freedom
        self.dof = dof
        
        # Dimensions
        n = dof * 2 
        m = dof  
        p = dof * 2
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, n, m, p)
        
        # Name
        self.name = str(dof) + 'DoF Manipulator'
        
        # Labels, bounds and units
        for i in range(dof):
            # joint angle states
            self.x_ub[i] = + np.pi * 2
            self.x_lb[i] = - np.pi * 2
            self.state_label[i] = 'Angle '+ str(i)
            self.state_units[i] = '[rad]'
            # joint velocity states
            self.x_ub[i+dof] = + np.pi * 2
            self.x_lb[i+dof] = - np.pi * 2
            self.state_label[i+dof] = 'Velocity ' + str(i)
            self.state_units[i+dof] = '[rad/sec]'
        for i in range(dof):
            self.input_label[i] = 'Torque ' + str(i)
            self.input_units[i] ='[Nm]'
        for i in range(dof):
            # joint angle states
            self.output_label[i] = 'Angle '+ str(i)
            self.output_units[i] = '[rad]'
            # joint velocity states
            self.output_label[i+dof] = 'Velocity ' + str(i)
            self.output_units[i+dof] = '[rad/sec]'
            
    ###########################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################
    
    ###########################################################################
    def H(self, q ):
        """ 
        Inertia matrix of the manipulator
        ----------------------------------
        dim( H ) = ( dof , dof )
        
        such that --> Kinetic Energy = 0.5 * dq^T * H(q) * dq
        
        """  
        
        H = np.diag( np.ones( self.dof ) ) # Default is identity matrix
        
        return H
    
    ###########################################################################
    def C(self, q , dq ):
        """ 
        Corriolis and Centrifugal Matrix 
        ------------------------------------
        dim( C ) = ( dof , dof )
        
        such that --> d H / dt =  C + C^T
        
        
        """ 
        
        C = np.zeros( ( self.dof , self.dof ) ) # Default is zeros matrix
        
        return C
    
    ###########################################################################
    def B(self, q ):
        """ 
        Actuator Matrix  : dof x m
        """
        
        B = np.diag( np.ones( self.dof ) ) # Default is identity matrix
        
        return B
    
    ###########################################################################
    def g(self, q ):
        """ 
        Gravitationnal forces vector : dof x 1
        """
        
        g = np.zeros( self.dof ) # Default is zero vector
        
        return g
    
    ###########################################################################
    # No need to overwrite the following functions for custom manipulators
    ###########################################################################
    
    #############################
    def x2q( self, x ):
        """ from state vector (x) to angle and speeds (q,dq) """
        
        q  = x[ 0        : self.dof ]
        dq = x[ self.dof : self.n   ]
        
        return [ q , dq ]
        
        
    #############################
    def q2x( self, q , dq ):
        """ from angle and speeds (q,dq) to state vector (x) """
        
        x = np.zeros( self.n )
        
        x[ 0        : self.dof ] = q
        x[ self.dof : self.n   ] = dq
        
        return x
    
    ##############################
    def generalized_forces(self, q  , dq  , ddq , t = 0 ):
        """ Computed generalized forces given a trajectory (inverse dynamic) """  
        
        H = self.H( q )
        C = self.C( q , dq )
        g = self.g( q )
                
        # Generalized forces
        forces = np.dot( H , ddq ) + np.dot( C , dq ) + g
        
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
        """ Computed accelerations given actuator forces u """  
        
        H = self.H( q )
        C = self.C( q , dq )
        g = self.g( q )
        B = self.B( q )
        
        ddq = np.dot( np.linalg.inv( H ) ,  ( np.dot( B , u )  - np.dot( C , dq ) - g ) )
        
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
        
        [ q , dq ] = self.x2q( x )       # from state vector (x) to angle and speeds (q,dq)
        
        ddq = self.ddq( q , dq , u , t ) # compute joint acceleration 
        
        dx = self.q2x( dq , ddq )        # from angle and speeds diff (dq,ddq) to state vector diff (dx)
        
        return dx
    
    
    ###########################################################################
    def kinetic_energy(self, q  , dq ):
        """ Compute kinetic energy of manipulator """  
        
        e_k = 0.5 * np.dot( dq , np.dot( self.H( q ) , dq ) )
        
        return e_k
