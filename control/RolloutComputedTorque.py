# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 14:59:30 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import RminComputedTorque   as RCTC


import numpy as np


'''
################################################################################
'''

    
class RolloutTorqueController( RCTC.RminComputedTorqueController ):
    """ Feedback law  """
    ############################
    def __init__( self , R = HM.HybridTwoLinkManipulator() ):
        """ """
        
        RCTC.RminComputedTorqueController.__init__( self , R  )
        
        
        self.FixCtl = RminCTC.RfixComputedTorqueController( R )
        
        # Copy traj & ddq_r functions
        self.FixCtl.get_traj         = self.get_traj
        self.FixCtl.compute_ddq_r    = self.compute_ddq_r
        
    
    ############################
    def u_star( self , ddq_r , x  , t ):
        """ 
        
        Compute optimal u given desired accel and actual states
        
        """
        
        # Cost is Q
        Q = np.zeros( self.n_gears )
        
        #for all gear ratio options
        for i in xrange( self.n_gears ):
            
            Q[i] = i # TO implement Rollout here
            
        
        # Optimal dsicrete mode
        i_star = 0
                        
        # Hysteresis
        if self.hysteresis:
            
            # if optimal gear is new
            if not(i_star == self.last_gear_i ):
                
                if ( t < self.min_delay + self.last_shift_t ):
                    
                    # Keep old gear ratio
                    i_star = self.last_gear_i
                                       
                # ok to gear-shift    
                else:
                    
                    self.last_gear_i  = i_star
                    self.last_shift_t = t
                    
        
        # Computed torque
        T_final = self.computed_torque( ddq_r , x , self.uD(i_star) ) 
        
        u  = np.append( T_final , self.uD( i_star ) )
        
        return u
        

        
        

    


        
        