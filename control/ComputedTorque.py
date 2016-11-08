# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 14:59:30 2016

@author: alex
"""

from AlexRobotics.dynamic    import Manipulator                     as M
from AlexRobotics.estimation import ManipulatorDisturbanceObserver  as OBS


from scipy.interpolate import interp1d

import numpy as np


'''
################################################################################
'''

    
class ComputedTorqueController:
    """ Feedback law  """
    ############################
    def __init__( self , R = M.TwoLinkManipulator() ):
        """ """
        
        self.R    = R  # Model of the robot used by the controller
        
        # Params
        self.w0   = 10
        self.zeta = 0.7 
        
        # default is fixed goal at zero
        self.goal        = np.zeros( R.n )
        self.traj_loaded = False
        self.ctl         = self.fixed_goal_ctl
        
        # Or load a solution: self.solution  = [ x , u , t , dx ]
        
        #self.traj_ref_pts = 'closest'
        self.traj_ref_pts = 'interpol'
        
        # For manual acc control
        self.ddq_manual_setpoint = np.zeros( self.R.dof )
        
        # Integral action with dist observer (beta)
        self.dist_obs_active = False
        self.obs             = OBS.DistObserver( R )
    
    
    ############################
    def ctl( self ,  x , t = 0 ):
        """
        
        Place holder for feedback law
        
        """
        
        pass
    
    
    ############################
    def traj_following_ctl( self , x , t = 0 ):
        """ 
        
        Given desired loaded trajectory and actual state, compute torques
        
        """
        
        ddq_d , dq_d , q_d = self.get_traj( t )

        ddq_r              = self.compute_ddq_r( ddq_d , dq_d , q_d , x )
        
        F                  = self.computed_torque( ddq_r , x , t )
        
        return F
        
        
    ############################
    def fixed_goal_ctl( self , x , t = 0 ):
        """ 
        
        Given desired fixed goal state and actual state, compute torques
        
        """
        
        ddq_d          =   np.zeros( self.R.dof )

        [ q_d , dq_d ] = self.R.x2q( self.goal  )   # from state vector (x) to angle and speeds (q,dq)

        ddq_r          = self.compute_ddq_r( ddq_d , dq_d , q_d , x )
        
        F              = self.computed_torque( ddq_r , x , t )
        
        return F
        
        
    ############################
    def manual_acc_ctl( self , x , t = 0 ):
        """ 
        
        Given desired acc, compute torques
        
        """

        ddq_r          = self.ddq_manual_setpoint
        
        F              = self.computed_torque( ddq_r , x , t )
        
        return F
    
        
    ############################
    def computed_torque( self , ddq_r , x , t ):
        """ 
        
        Given actual state, compute torque necessarly for a given acceleration vector
        
        """
        
        [ q , dq ] = self.R.x2q( x )   # from state vector (x) to angle and speeds (q,dq)
        
        F = self.R.F( q , dq , ddq_r ) # Generalized force necessarly
        
        # Dist obs:
        if self.dist_obs_active:
            self.obs.update_estimate(  x , F , t )
            self.R.f_dist_steady = self.obs.f_ext_hat
        
        return F
        
        
    ############################
    def compute_ddq_r( self , ddq_d , dq_d , q_d , x ):
        """ 
        
        Given desired trajectory and actual state, compute ddq_r
        
        """
        
        [ q , dq ] = self.R.x2q( x )   # from state vector (x) to angle and speeds (q,dq)
        
        q_e   = q  -  q_d
        dq_e  = dq - dq_d
        
        ddq_r = ddq_d - 2 * self.zeta * self.w0 * dq_e - self.w0 ** 2 * q_e
        
        # Save for Debug
        self.q_e   = q_e
        self.dq_e  = dq_e
        self.ddq_r = ddq_r
        
        return ddq_r
        
        
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
        
        # assign new controller
        self.ctl = self.traj_following_ctl
        
        # Create interpol functions
        self.q   = interp1d(t,q)
        self.dq  = interp1d(t,dq)
        self.ddq = interp1d(t,ddq)
        
    
    ############################
    def get_traj( self , t  ):
        """ 
        
        Find closest point on the trajectory
        
        """
        
        if t < self.max_time - 0.1 :
            
            if self.traj_ref_pts == 'interpol':
            
                # Load trajectory
                q     = self.q(   t )
                dq    = self.dq(  t )
                ddq   = self.ddq( t )          
            
            elif self.traj_ref_pts == 'closest':
            
                # Find closet index
                times = self.traj[3]
                i     = (np.abs(times - t)).argmin() + 1
                
                # Load trajectory
                ddq = self.traj[0][:,i]
                dq  = self.traj[1][:,i]
                q   = self.traj[2][:,i]
            
        else:
            
            # Fixed goal
            ddq = np.zeros( self.R.dof )
            dq  = self.goal[ self.R.dof : 2 * self.R.dof]
            q   = self.goal[ 0          : self.R.dof    ]
            
        
        return ddq , dq , q
        
        

'''
################################################################################
'''

    
class SlidingModeController( ComputedTorqueController ):
    """ Feedback law  """
    
    
    ############################
    def __init__( self , R = M.TwoLinkManipulator() ):
        """ """
        
        ComputedTorqueController.__init__( self , R  )
        
        
        # Params
        
        self.lam = 1  # Sliding surface slope
        self.D   = 1  # Discontinuous gain
        
        
        
        
    ############################
    def compute_sliding_variables( self , ddq_d , dq_d , q_d , x ):
        """ 
        
        Given desired trajectory and actual state
        
        """
        
        [ q , dq ] = self.R.x2q( x )   # from state vector (x) to angle and speeds (q,dq)
        
        q_e   = q  -  q_d
        dq_e  = dq - dq_d
        
        s      = dq_e  + self.lam * q_e
        dq_r   = dq_d  - self.lam * q_e
        ddq_r  = ddq_d - self.lam * dq_e
        
        #ddq_r = ddq_d - 2 * self.zeta * self.w0 * dq_e - self.w0 ** 2 * q_e
        
        # Save for Debug
        self.q_e   = q_e
        self.dq_e  = dq_e
        self.ddq_r = ddq_r
        
        return [ s , dq_r , ddq_r ]
        
        
    ############################
    def K( self , q ,t  ):
        """ Discontinuous gain matrix """
        
        K = np.diag( np.ones( self.R.dof ) ) * self.D
        
        return K
        
        
    ############################
    def sliding_torque( self , ddq_r , s , x , t ):
        """ 
        
        Given actual state, compute torque necessarly to guarantee convergence
        
        """
        
        [ q , dq ] = self.R.x2q( x )     # from state vector (x) to angle and speeds (q,dq)
        
        F_computed      = self.R.F( q , dq , ddq_r )   # Generalized force necessarly
        
        F_discontinuous = np.dot( self.K( q , t ) ,  np.sign( s ) )
        
        F_tot = F_computed - F_discontinuous
        
        return F_tot
        
        
        
        
    ############################
    def traj_following_ctl( self , x , t = 0 ):
        """ 
        
        Given desired loaded trajectory and actual state, compute torques
        
        """
        
        ddq_d , dq_d , q_d    = self.get_traj( t )

        [ s , dq_r , ddq_r ]  = self.compute_sliding_variables( ddq_d , dq_d , q_d , x )
        
        F                     = self.sliding_torque( ddq_r , s , x , t )
        
        return F
        
        
    ############################
    def fixed_goal_ctl( self , x , t = 0 ):
        """ 
        
        Given desired fixed goal state and actual state, compute torques
        
        """
        
        ddq_d          =   np.zeros( self.R.dof )

        [ q_d , dq_d ] = self.R.x2q( self.goal  )   # from state vector (x) to angle and speeds (q,dq)

        [ s , dq_r , ddq_r ]  = self.compute_sliding_variables( ddq_d , dq_d , q_d , x )
        
        F                     = self.sliding_torque( ddq_r , s , x , t )
        
        return F
        
        
       