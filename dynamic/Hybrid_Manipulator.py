# -*- coding: utf-8 -*-
"""
Created on Sat Mar 19 20:37:16 2016

@author: alex
"""

import matplotlib.animation as animation
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from AlexRobotics.dynamic import DynamicSystem as RDDS
from AlexRobotics.dynamic import Manipulator   as M



#########################################################################################################   
#########################################################################################################       
        
class HybridOneLinkManipulator( M.OneLinkManipulator ) :
    """ 2DOF Manipulator Class """
    
    
    ############################
    def __init__(self):
        
        n = 2
        m = 2
        
        M.OneLinkManipulator.__init__(self, n , m )
        
        self.state_label = ['Angle','Speed']
        self.input_label = ['Torque','Ratop']
        
        self.state_units = ['[rad]','[rad/sec]']
        self.input_units = ['[Nm]','']
        
        self.x_ub = np.array([ 2*np.pi , 2*np.pi])    # States Upper Bounds
        self.x_lb = np.array([-2*np.pi, -2*np.pi])    # States Lower Bounds
        
        tmax = 10
        
        self.u_ub = np.array([ tmax , 10])      # Control Upper Bounds
        self.u_lb = np.array([-tmax , 1 ])      # Control Lower Bounds
        
        # Default State and inputs        
        self.xbar = np.zeros(n)
        self.ubar = np.array([0,1])
        
        self.setparams()
    
        self.Ia = 1
        self.Da = 1

        
    ##############################
    def Tlosses(self, dq = np.zeros(1) , ddq = np.zeros(1)):
        """ Computed torques losses given a trajectory  """  
                
        T = np.dot( self.Ia  , ddq ) + np.dot( self.Da  , dq )
        
        return T
        
        
    ##############################
    def T(self, q = np.zeros(1) , dq = np.zeros(1) , ddq = np.zeros(1) , R = 1 ):
        """ Computed acutator torques given a trajectory and gear ratio """ 
        
        F = self.F( q , dq , ddq )
        
        Tl = self.Tlosses( dq , ddq )
        
        T = np.dot( 1. / R , F ) + np.dot( R , Tl ) 
        
        return T
        
        
    ##############################
    def ddq_a(self, q = np.zeros(1) , dq = np.zeros(1) , T = np.zeros(1) , R = 1 , d = 0 ):
        """ Computed accelerations given actuator torques and gear ratio """  
        
        
        Ha = self.H( q ) + np.dot( R , np.dot( R , self.Ia ) )
        
        C  = self.C( q , dq )
        D  = self.D( q , dq )
        
        Ca =  C + D + np.dot( R , np.dot( R , self.Da ) )
        
        G  = self.G( q )
        
        ddq = np.dot( 1. / Ha  ,  ( np.dot( R , T ) - np.dot( Ca , dq ) - G  + d ) )
        
        return ddq
        
        
    #############################
    def fc(self, x = np.zeros(2) , u = np.array([0,1]) , t = 0 ):
        """ 
        Continuous time function evaluation
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        dx = np.zeros(self.n) # State derivative vector
        
        q  = x[0]
        dq = x[1]
        
        ddq = self.ddq_a( q , dq , u[0] , u[1] )
        
        dx[0] = dq
        dx[1] = ddq
        
        return dx
        
        
      
 #########################################################################################################   
#########################################################################################################  
        


class HybridTwoLinkManipulator( M.TwoLinkManipulator ) :
    """ 2DOF Manipulator Class """
    
    
    ############################
    def __init__(self, n = 4 , m = 3 ):
        
        M.TwoLinkManipulator.__init__(self, n , m-1 )
        
        self.n = n
        self.m = m        
        
        self.input_label = ['Torque 1','Torque 2','Mode']
        self.input_units = ['[Nm]','[Nm]','']
        
        tmax = 5
        
        self.u_ub = np.array([ tmax, tmax, 3])      # Control Upper Bounds
        self.u_lb = np.array([-tmax,-tmax, 0])      # Control Lower Bounds
        
        self.ubar = np.array([0,0,0])
        
        self.setActuatorParams()
        

    #############################
    def setActuatorParams(self, r1 = 1 , r2 = 10 , ja = 1 , ba = 1 ):
        """ Set model parameters here """
        
        # Gear ratio
        self.R = [np.diag([r1,r1]),np.diag([r1,r2]),np.diag([r2,r1]),np.diag([r2,r2])]
        
        # Inertia
        self.Ia = np.diag([ja,ja])
        
        # Damping (linear)
        self.Da = np.diag([ba,ba])
        
        
    ##############################
    def Tlosses(self, dq = np.zeros(2) , ddq = np.zeros(2)):
        """ Computed torques losses given a trajectory  """  
                
        T = np.dot( self.Ia  , ddq ) + np.dot( self.Da  , dq )
        
        return T
        
        
    ##############################
    def T(self, q = np.zeros(2) , dq = np.zeros(2) , ddq = np.zeros(2) , u3 = 0 ):
        """ Computed acutator torques given a trajectory and gear ratio """ 
        
        R = self.R[u3]
        
        F = self.F( q , dq , ddq )
        
        Tl = self.Tlosses( dq , ddq )
        
        T = np.dot( np.linalg.inv(R) , F ) + np.dot( R , Tl ) 
        
        return T
        
        
    ##############################
    def ddq_a(self, q = np.zeros(2) , dq = np.zeros(2) , T = np.zeros(2) , u3 = 0 ):
        """ Computed accelerations given actuator torques and gear ratio """  
        
        R = self.R[ int(u3) ]
        
        Ha = self.H( q ) + np.dot( R , np.dot( R , self.Ia ) )
        
        C  = self.C( q , dq )
        D  = self.D( q , dq )
        
        Ca =  C + D + np.dot( R , np.dot( R , self.Da ) )
        
        
        G  = self.G( q )
        
        ddq = np.dot( np.linalg.inv( Ha ) ,  ( np.dot( R , T ) - np.dot( Ca , dq ) - G ) )
        
        return ddq
        
        
    #############################
    def fc(self, x = np.zeros(4) , u = np.zeros(3) , t = 0 ):
        """ 
        Continuous time function evaluation
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        dx = np.zeros(self.n) # State derivative vector
        
        q  = x[0:2]
        dq = x[2:4]
        
        ddq = self.ddq_a( q , dq , u[0:2] , u[2] )
        
        dx[0:2] = dq
        dx[2:4] = ddq
        
        return dx
        
        
    ##############################
    def e_kinetic(self, q = np.zeros(2) , dq = np.zeros(2) , u3 = 0 ):
        """ Compute kinetic energy of manipulator """  
        
        R = self.R[ int(u3) ]
        
        Ha = self.H( q ) + np.dot( R , np.dot( R , self.Ia ) )        
        
        e_k = 0.5 * np.dot( dq , np.dot( Ha , dq ) )
        
        return e_k
        