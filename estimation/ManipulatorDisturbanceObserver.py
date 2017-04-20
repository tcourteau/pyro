# -*- coding: utf-8 -*-


import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from AlexRobotics.dynamic import DynamicSystem        as RDDS
from AlexRobotics.dynamic import Manipulator          as M
from AlexRobotics.dynamic import Hybrid_Manipulator   as HM


'''
################################################################################
'''

    
class DistObserver:
    """ Integral Action for COmputed Torque """
    ############################
    def __init__( self , R ):
        """ """
        
        self.R          = R  # Model of the robot used by the controller
        
        self.ishybrid   = False
        
        # INIT
        self.f_ext_hat  = np.zeros( self.R.dof ) # F_ext estimate
        self.init       = True
        
    
    ############################
    def update_estimate( self , x , u , t ):
        
        [ q , dq ] = self.R.x2q( x )
        
        # Acc prediction with no disturbance
        self.R.f_dist_steady = np.zeros( self.R.dof )
        dx_hat               = self.R.fc( x , u , t )
        [ dq_hat , ddq_hat ] = self.R.x2q( dx_hat )  
        
        if self.init:
            
            # First time calling the function
            self.dq_last     = dq       # save speed of last step
            self.ddq_hat     = ddq_hat  # predicted acc
            self.t_last      = t
            self.init        = False
            
        else:
            
            # Steady state
            dt       = (  t - self.t_last  )
            ddq_real = ( dq - self.dq_last ) / dt
            ddq_e    = self.ddq_hat - ddq_real
            
            # Inertia Matrix
            if self.ishybrid:
                R    = u[ self.R.dof ]
                H    = self.R.H_all( q , R )
            else:
                H    = self.R.H( q )
                
            # Unexplained gen. forces based on acc mismatch
            dist = np.dot( H , ddq_e ) # point estimate
            
            # Filter
            self.f_ext_hat = 0.90 * self.f_ext_hat + 0.1 * dist
            
            # Debug
            print('Dist: ', self.f_ext_hat )
            
            # Memory
            self.dq_last     = dq       # save speed of last step
            self.ddq_hat     = ddq_hat  # predicted acc
            self.t_last      = t