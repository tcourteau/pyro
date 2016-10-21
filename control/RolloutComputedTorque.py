# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 14:59:30 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import RminComputedTorque   as RCTC

from AlexRobotics.dynamic  import DynamicSystem        as DS


import numpy as np


'''
################################################################################
'''

    
class RolloutComputedTorqueController( RCTC.RminComputedTorqueController ):
    """ Feedback law  """
    ############################
    def __init__( self , R = HM.HybridTwoLinkManipulator() ):
        """ """
        
        RCTC.RminComputedTorqueController.__init__( self , R  )
        
        
        self.FixCtl = RCTC.RfixComputedTorqueController( R )
        
        # Copy traj & ddq_r functions
        self.FixCtl.get_traj         = self.get_traj
        self.FixCtl.compute_ddq_r    = self.compute_ddq_r
        
        # Assign Rollout Controller
        self.R.ctl = self.FixCtl.ctl
        
        
        self.horizon = 1
        self.sim_dt  = 0.1
        self.sim_n   =  self.horizon / self.sim_dt + 1
        
    
    ############################
    def u_star( self , ddq_r , x  , t ):
        """ 
        
        Compute optimal u given desired accel and actual states
        
        """
        
        # Cost is Q
        Q = np.zeros( self.n_gears )
        
        #for all gear ratio options
        for i in xrange( self.n_gears ):
            
            # Assign the fixed gear
            self.FixCtl.R_index = i
            
            # Simulate from t to (t+1) with this controller
            Q[i] = self.Rollout( x , t )
            
        
        # Optimal dsicrete mode
        i_star = Q.argmin()
        
        print Q , i_star
                        
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
        
        
        
    ############################
    def Rollout( self , x  , t ):
        
        self.Sim    = DS.Simulation( self.R , t + self.horizon ,  self.sim_n )
        
        self.Sim.J  = 0
        self.Sim.x0 = x
        self.Sim.t0 = t
        
        # Cost: torque quadratic cost 
        self.Sim.Q = np.zeros( ( self.R.n , self.R.n ) )
        self.Sim.H = np.zeros( ( self.R.n , self.R.n ) )
        self.Sim.R = np.diag( np.append( np.ones( self.R.dof ) , 0 ) )
        
        self.Sim.domain_check     = True
        self.Sim.domain_fail_cost = 1
        
        #Compute
        self.Sim.compute()
        
        #self.Sim.plot_CL()
        #raw_input('Press <ENTER> to continue')
        
        return self.Sim.J
        
        
        

        
        

    


        
        