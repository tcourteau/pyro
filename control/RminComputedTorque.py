# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 14:59:30 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import ComputedTorque       as CTC


import numpy as np


'''
################################################################################
'''

    
class RminComputedTorqueController( CTC.ComputedTorqueController ):
    """ Feedback law  """
    ############################
    def __init__( self , R = HM.HybridTwoLinkManipulator() ):
        """ """
        
        CTC.ComputedTorqueController.__init__( self , R  )
        
        self.n_gears = 4
        
        self.hysteresis       = False
        self.hys_level        = 1
        self.last_gear_i      = 0 # Default gear
        self.min_delay        = -10000   # default is not constraint
        self.last_shift_t     = -1
        
        # Integral action with dist observer (beta)
        self.dist_obs_active = False
        self.obs             = ExtDistObserver( R )
        
    ############################
    def reset_hysteresis( self ):
        """ Reset all memorized info in controlled, ex: before restarting a simulation """
        
        self.last_gear_i      = 0 # Default gear
        self.last_shift_t     = 0
        
        
    ############################
    def traj_following_ctl( self , x , t ):
        """ 
        
        Given desired loaded trajectory and actual state, compute torques and optimal gear ratio
        
        """
        
        ddq_d , dq_d , q_d = self.get_traj( t )

        ddq_r              = self.compute_ddq_r( ddq_d , dq_d , q_d , x )
        
        u                  = self.u_star( ddq_r , x , t )
        
        
        return u
        
        
    ############################
    def fixed_goal_ctl( self , x , t = 0 ):
        """ 
        
        Given desired fixed goal state and actual state, compute torques and optimal gear ratio
        
        """
        
        ddq_d          =   np.zeros( self.R.dof )

        [ q_d , dq_d ] = self.R.x2q( self.goal  )   # from state vector (x) to angle and speeds (q,dq)

        ddq_r          = self.compute_ddq_r( ddq_d , dq_d , q_d , x )
        
        u              = self.u_star( ddq_r , x , t )
        
        # Disturbance Observer Test
        if self.dist_obs_active:
            self.obs.update_estimate( x , u , t )
            self.R.ext_cst_force = self.obs.f_ext_hat
        
        return u
        
        
    ############################
    def manual_acc_ctl( self , x , t = 0 ):
        """ 
        
        Given desired acc, compute torques and optimal gear ratio
        
        """

        ddq_r          = self.ddq_manual_setpoint
        
        u              = self.u_star( ddq_r , x , t )
        
        return u
        
    
    ############################
    def u_star( self , ddq_r , x  , t ):
        """ 
        
        Compute optimal u given desired accel and actual states
        
        """
        
        # Cost is Q
        Q = np.zeros( self.n_gears )
        T = np.zeros( ( self.n_gears , self.R.dof ) )
        
        #for all gear ratio options
        for i in xrange( self.n_gears ):
            
            T[i] = self.computed_torque( ddq_r , x , self.uD(i) ) 
            
            # Cost is norm of torque
            #Q[i] = np.dot( T[i] , T[i] )
            
            # Verify validity
            u_test  = np.append( T[i] , self.uD(i) )
            
            if self.R.isavalidinput( x , u_test ):
                # valid option
                
                # Cost is norm of torque
                Q[i] = np.dot( T[i] , T[i] )
                
            else:
                
                # Bad option
                Q[i] = 9999999999 # INF
                #print 'bad option'
            
        
        # Optimal dsicrete mode
        i_star = Q.argmin()
        
        #print Q , i_star , t , x
                        
        # Hysteresis
        if self.hysteresis:
            
            # if optimal gear is new
            if not(i_star == self.last_gear_i ):
                
                gear_shift_gain = np.linalg.norm( T[ i_star ] - T[ self.last_gear_i ] )
                
                # if gain of changing is small
                if gear_shift_gain < self.hys_level :
                    
                    # Keep old gear ratio
                    i_star = self.last_gear_i
                    
                    #print 'shifting not Allowed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                    #print 'too small gain       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                
                # if changed recently
                elif ( t < self.min_delay + self.last_shift_t ):
                    
                    # Keep old gear ratio
                    i_star = self.last_gear_i
                    
                    #print 'shifting not Allowed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                    #print 'too sshort delay     !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                    #print '  t:', t , '  timing:' , self.min_delay + self.last_shift_t 
                    
                # ok to gear-shift    
                else:
                    
                    self.last_gear_i  = i_star
                    self.last_shift_t = t
                    
            
        u  = np.append( T[ i_star ] , self.uD( i_star ) )
        
        return u
        

    ############################
    def computed_torque( self , ddq_r , x , R ):
        """ 
        
        Given actual state and gear ratio, compute torque necessarly for a given acceleration vector
        
        """
        
        [ q , dq ] = self.R.x2q( x )   # from state vector (x) to angle and speeds (q,dq)
        
        F = self.R.T( q , dq , ddq_r , R ) # Generalized force necessarly
        
        return F
        
        
    ############################
    def uD( self , i  ):
        """ 
        
        Return the discrete value for the gear ratio
        
        1-Dof # -> input is directly the ratio
        
        else  # -> input is the index
        
        """
        
        if self.R.dof == 1 :
            
            return self.R.R[ i ]
            
        else:
            
            return i
        
        

    

'''
################################################################################
'''

    
class RfixComputedTorqueController( RminComputedTorqueController ):
    """ Feedback law  """
    ############################
    def __init__( self , R = HM.HybridTwoLinkManipulator() , R_index = 0 ):
        """ """
        
        CTC.ComputedTorqueController.__init__( self , R  )
        
        self.R_index = R_index    # Fixed gear ratio index
        
        # Integral action with dist observer (beta)
        self.dist_obs_active = False
        self.obs             = ExtDistObserver( R )

        
    ############################
    def reset_hysteresis( self ):
        """ Reset all memorized info in controlled, ex: before restarting a simulation """
        
        pass
        

    
    ############################
    def u_star( self , ddq_r , x  , t ):
        """      
        only one gear option
        """
        
        Ratio  = self.uD( self.R_index )
        
        Torque = self.computed_torque( ddq_r , x , Ratio ) 
         
        u  = np.append( Torque , Ratio )
        
        return u
        

        
        


        