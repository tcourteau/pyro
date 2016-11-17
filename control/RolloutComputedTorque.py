# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 14:59:30 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import RminComputedTorque   as RCTC

from AlexRobotics.dynamic  import DynamicSystem        as DS

from scipy.interpolate import interp1d
import numpy as np


'''
################################################################################
'''


###############################################################################    
### Rollout with base policy: Computed Torque with fixed gear
###############################################################################
    
class RolloutComputedTorqueController( RCTC.RminComputedTorqueController ):
    """ Feedback law  """
    ############################
    def __init__( self , R = HM.HybridTwoLinkManipulator() ):
        """ """
        
        RCTC.RminComputedTorqueController.__init__( self , R  )
        
        
        self.FixCtl = RCTC.RfixComputedTorqueController( R )
        

        # Assign Rollout Controller to Simulation Model
        self.R.ctl = self.FixCtl.ctl
        
        
        self.horizon = 1
        self.sim_dt  = 0.02
        self.sim_n   =  self.horizon / self.sim_dt + 1
        
        
        # Domain check in simulation?
        self.domain_check     = True
        self.domain_fail_cost = 1
               
    
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
        
        #print Q , i_star, t , x
                        
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
        
        
        self.sim_n  =  self.horizon / self.sim_dt + 1
        
        self.Sim    = DS.Simulation( self.R , t + self.horizon ,  self.sim_n , 'euler' )
        
        self.Sim.J  = 0
        self.Sim.x0 = x
        self.Sim.t0 = t
        
        # Cost: torque quadratic cost 
        self.Sim.Q = np.zeros( ( self.R.n , self.R.n ) )
        self.Sim.H = np.zeros( ( self.R.n , self.R.n ) )
        self.Sim.R = np.diag( np.append( np.ones( self.R.dof ) , 0 ) )
        
        self.Sim.domain_check     = self.domain_check
        self.Sim.domain_fail_cost = self.domain_fail_cost
        
        #Compute
        self.Sim.compute()
        
        #self.Sim.plot_CL()
        #raw_input('Press <ENTER> to continue')
        
        return self.Sim.J
        
        
    
    ############################
    def load_trajectory( self , solution  ):
        """ 
        
        Load Open-Loop trajectory solution to use as reference trajectory
        
        """
        
        self.solution = solution
        
        q   = solution[0][ 0          :     self.R.dof , : ]
        dq  = solution[0][ self.R.dof : 2 * self.R.dof , : ]
        ddq = solution[3][ self.R.dof : 2 * self.R.dof , : ]
        t   = solution[2]
        
        self.traj = [ ddq , dq , q , t ]
        
        self.max_time = t.max()
        
        # assign traj following to both global and base controller
        self.ctl           = self.traj_following_ctl
        self.FixCtl.ctl    = self.FixCtl.traj_following_ctl
        self.R.ctl         = self.FixCtl.ctl
        
        # Note : very slow when using traj following?? interpolation is taking time? because fixed goal is fast
        # Sloved by switching to euler integration
        
        # Create interpol functions
        self.q   = interp1d(t,q)
        self.dq  = interp1d(t,dq)
        self.ddq = interp1d(t,ddq)
        
        # Same traj for Base policy
        self.assign_traj_functions()
        
        
    ############################
    def assign_traj_functions( self ):
        
        
        # Copy traj & ddq_r functions for base policy
        self.FixCtl.get_traj         = self.get_traj
        self.FixCtl.compute_ddq_r    = self.compute_ddq_r
        
        
        

        
        

###############################################################################    
### Rollout with base policy: Sliding Mode with fixed gear
###############################################################################
    
class RolloutSlidingModeController( RCTC.RminSlidingModeController ,  RolloutComputedTorqueController ):
    """ Feedback law  """
    ############################
    def __init__( self , R = HM.HybridTwoLinkManipulator() ):
        """ """
        
        RCTC.RminSlidingModeController.__init__( self , R  )
        
        
        self.FixCtl = RCTC.RfixSlidingModeController( R )
        

        # Assign Rollout Controller to Simulation Model
        self.R.ctl = self.FixCtl.ctl
        
        # Rollout Params
        self.horizon = 1
        self.sim_dt  = 0.02
        self.sim_n   =  self.horizon / self.sim_dt + 1
        
        # Domain check in simulation?
        self.domain_check     = True
        self.domain_fail_cost = 1
               
    
    ############################
    def u_star( self , ddq_r , x , s , t ):
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
            #Q[i] = self.Rollout( x , t )
            
            # Verify gear selection validity
            u_test  = np.append( np.zeros( self.R.dof ) , self.uD(i) )
            
            if self.R.isavalidinput( x , u_test ):
                # valid option
                
                # Simulate from t to (t+1) with this controller
                Q[i] = self.Rollout( x , t )
                
            else:
                
                # Bad option
                Q[i] = 9999999999 # INF
            
        
        # Optimal dsicrete mode
        i_star = Q.argmin()
        
        #print Q , i_star, t , x
                        
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
        #T_final = self.computed_torque( ddq_r , x , self.uD(i_star) ) 
        #u  = np.append( T_final , self.uD( i_star ) )
        
        self.FixCtl.R_index = i_star
        
        u = self.FixCtl.u_star( ddq_r , x , s , t )        
        
        return u
        
        
    ############################
    def assign_traj_functions( self ):
        
        
        # Copy traj & ddq_r functions for base policy
        self.FixCtl.get_traj                  = self.get_traj
        self.FixCtl.compute_sliding_variables = self.compute_sliding_variables
        

    

        