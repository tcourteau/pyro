# -*- coding: utf-8 -*-
"""
Created on Fri Nov  9 10:46:14 2018

@author: alxgr
"""

import numpy as np
from scipy.interpolate import interp1d

from AlexRobotics.control import controller
from AlexRobotics.dynamic import mechanical

###############################################################################
# Simple Computed Torque
###############################################################################
        
class ComputedTorqueController( controller.StaticController ) :
    """ 
    

    """    
    
    ############################
    def __init__(self, model = mechanical.MechanicalSystem() ):
        """
        
        ---------------------------------------
        r  : reference signal vector  k x 1
        y  : sensor signal vector     p x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        ---------------------------------------
        u = c( y , r , t )
        
        """
        
        self.model = model
        
        # Dimensions
        self.k = model.dof   
        self.m = model.m
        self.p = model.p
        
        controller.StaticController.__init__(self, self.k, self.m, self.p)
        
        # Label
        self.name = 'Computed Torque Controller'
        
        # Params
        self.w0   = 1
        self.zeta = 0.7 
        
    
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback static computation u = c(y,r,t)
        
        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        u = np.zeros(self.m) 
        
        x   = y 
        q_d = r
        
        u = self.fixed_goal_ctl( x , q_d , t )
        
        
        return u
    
        
        
    ############################
    def fixed_goal_ctl( self , x , q_d , t = 0 ):
        """ 
        
        Given desired fixed goal state and actual state, compute torques
        
        """
        
        ddq_d          =   np.zeros( self.model.dof )
        dq_d           =   np.zeros( self.model.dof )

        ddq_r          = self.compute_ddq_r( ddq_d , dq_d , q_d , x )
        
        u              = self.computed_torque( ddq_r , x , t )
        
        return u
        
    
        
    ############################
    def computed_torque( self , ddq_r , x , t ):
        """ 
        
        Given actual state, compute torque necessarly for a given acceleration vector
        
        """
        
        [ q , dq ] = self.model.x2q( x )   # from state vector (x) to angle and speeds (q,dq)
        
        u = self.model.actuator_forces( q , dq , ddq_r )
        
        return u
        
        
    ############################
    def compute_ddq_r( self , ddq_d , dq_d , q_d , x ):
        """ 
        
        Given desired trajectory and actual state, compute ddq_r
        
        """
        
        [ q , dq ] = self.model.x2q( x )   # from state vector (x) to angle and speeds (q,dq)
        
        q_e   = q  -  q_d
        dq_e  = dq - dq_d
        
        ddq_r = ddq_d - 2 * self.zeta * self.w0 * dq_e - self.w0 ** 2 * q_e
        
        return ddq_r
    


###############################################################################
# Simple Computed Torque
###############################################################################
        
class TrajectoryFollowingComputedTorqueController( ComputedTorqueController ) :
    """ 
    """
    
    ############################
    def __init__(self, model , traj ):
        """
        ---------------------------------------
        r  : reference signal vector  k x 1
        y  : sensor signal vector     p x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        ---------------------------------------
        u = c( y , r , t )
        """
        
        ComputedTorqueController.__init__( self , model )
        
        self.load_trajectory( traj )
        
        self.mode = 'interpol'
        
        
    ############################
    def load_trajectory( self , traj  ):
        """ 
        
        Load Open-Loop trajectory solution to use as reference trajectory
        
        """
        
        self.trajectory = traj
        
        q   = traj.x_sol[ :,    0           :     self.model.dof ]
        dq  = traj.x_sol[ :, self.model.dof : 2 * self.model.dof ]
        ddq = traj.dx_sol[:, self.model.dof : 2 * self.model.dof ]
        t   = traj.t_sol
        
        # Create interpol functions
        self.q   = interp1d(t,q.T)
        self.dq  = interp1d(t,dq.T)
        self.ddq = interp1d(t,ddq.T)
        
    ############################
    def get_traj( self , t  ):
        """ 
        
        Find closest point on the trajectory
        
        """
        
        if t < self.trajectory.time_final :

            # Load trajectory
            q     = self.q(   t )
            dq    = self.dq(  t )
            ddq   = self.ddq( t )          

        else:
            
            q     = self.rbar
            dq    = np.zeros( self.model.dof )
            ddq   = np.zeros( self.model.dof )
            
        
        return ddq , dq , q
    
    ############################
    def traj_following_ctl( self , x , t = 0 ):
        """ 
        
        Given desired loaded trajectory and actual state, compute torques
        
        """
        
        ddq_d , dq_d , q_d = self.get_traj( t )

        ddq_r              = self.compute_ddq_r( ddq_d , dq_d , q_d , x )
        
        u                  = self.computed_torque( ddq_r , x , t )
        
        return u
        
        
    #############################
    def c( self , y , r , t ):
        """ 
        Feedback static computation u = c(y,r,t)
        
        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        u = np.zeros(self.m) 
        
        x = y 
        
        u = self.traj_following_ctl( x , t )
        
        
        return u
    
    


'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    from AlexRobotics.dynamic import pendulum
    
    sp = pendulum.SinglePendulum()
    c  = ComputedTorqueController( sp )
    
    # New cl-dynamic
    clsp = controller.ClosedLoopSystem( sp ,  c )
    
    x0 = np.array([2,0])
    clsp.plot_phase_plane_trajectory( x0 )
    clsp.sim.plot('xu')
    clsp.animate_simulation()
    
    ####################################
    dp = pendulum.DoublePendulum()
    c2  = ComputedTorqueController( dp )
    
    # New cl-dynamic
    cldp = controller.ClosedLoopSystem( dp ,  c2 )
    
    x0 = np.array([2,1,0,0])
    cldp.plot_phase_plane_trajectory( x0 , 10 , 0 , 2)
    cldp.plot_phase_plane_trajectory( x0 , 10 , 1 , 3)
    cldp.sim.plot('xu')
    cldp.animate_simulation()
        
