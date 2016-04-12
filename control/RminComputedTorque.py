# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 14:59:30 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.control  import ComputedTorque       as CTC


from scipy.interpolate import interp1d

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
        
        
    ############################
    def computed_torque( self , ddq_r , x , R ):
        """ 
        
        Given actual state and gear ratio, compute torque necessarly for a given acceleration vector
        
        """
        
        q  = x[0:2]
        dq = x[2:4]
        
        F = self.R.T( q , dq , ddq_r , R ) # Generalized force necessarly
        
        return F
        
        
    ############################
    def compute_ddq_r( self , ddq_d , dq_d , q_d , x ):
        """ 
        
        Given desired trajectory and actual state, compute ddq_r
        
        """
        
        q  = x[0:2]
        dq = x[2:4]
        
        q_e   = q  -  q_d
        dq_e  = dq - dq_d
        
        ddq_r = ddq_d - 2 * self.zeta * self.w0 * dq_e - self.w0 ** 2 * q_e
        
        return ddq_r
        
        
    ############################
    def fixed_goal_ctl( self , x , t = 0 ):
        """ 
        
        Given desired fixed goal state and actual state, compute torques and optimal gear ratio
        
        """
        
        ddq_d = np.zeros(2)
        dq_d  = self.goal[2:4]
        q_d   = self.goal[0:2]

        ddq_r = self.compute_ddq_r( ddq_d , dq_d , q_d , x )
        
        
        # Cost is Q
        Q = np.zeros( self.n_gears )
        
        #for all gear ratio options
        for i in xrange( self.n_gears ):
            
            F = self.computed_torque( ddq_r , x , i )
            
            # Cost is norm of torque
            Q[i] = np.dot( F , F )
            
            
        R_star = Q.argmin()
            
        F = self.computed_torque( ddq_r , x , R_star )
        
        u = np.append( F , R_star )
        
        return u
        
        
    ############################
    def load_trajectory( self , solution  ):
        """ 
        
        Load Open-Loop trajectory solution to use as reference trajectory
        
        """
        
        self.solution = solution
        
        q   = solution[0][0:2,:]
        dq  = solution[0][2:4,:]
        ddq = solution[3][2:4,:]
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
            ddq = np.zeros(2)
            dq  = self.goal[2:4]
            q   = self.goal[0:2]
            
        
        return ddq , dq , q
        
    
    ############################
    def traj_following_ctl( self , x , t ):
        """ 
        
        Given desired loaded trajectory and actual state, compute torques and optimal gear ratio
        
        """
        
        ddq_d , dq_d , q_d = self.get_traj( t )

        ddq_r              = self.compute_ddq_r( ddq_d , dq_d , q_d , x )
        
        # Cost is Q
        Q = np.zeros( self.n_gears )
        
        #for all gear ratio options
        for i in xrange( self.n_gears ):
            
            F = self.computed_torque( ddq_r , x , i )
            
            # Cost is norm of torque
            Q[i] = np.dot( F , F )
            
            
        R_star = Q.argmin()
            
        F = self.computed_torque( ddq_r , x , R_star )
        
        u = np.append( F , R_star )
        
        return u

        
        