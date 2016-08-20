# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 13:09:20 2015

@author: agirard
"""

import matplotlib.animation as animation
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from AlexRobotics.dynamic import DynamicSystem as RDDS


'''
#################################################################
##################       2DOF Manipulator Class          ########
#################################################################
'''



class TwoLinkManipulator( RDDS.DynamicSystem ) :
    """ Manipulator base-class """
    
    
    ############################
    def __init__(self, n = 4 , m = 2 ):
        
        RDDS.DynamicSystem.__init__(self, n , m )
        
        self.dof     = 2 # number of degree of freedoms
                
        self.state_label = ['Angle 1','Angle 2','Speed 1','Speed 2']
        self.input_label = ['Torque 1','Torque 2']
        
        self.state_units = ['[rad]','[rad]','[rad/sec]','[rad/sec]']
        self.input_units = ['[Nm]','[Nm]']
        
        self.x_ub = np.array([ 6, 6, 6, 6])    # States Upper Bounds
        self.x_lb = np.array([-6,-6,-6,-6])    # States Lower Bounds
        
        tmax = 1
        
        self.u_ub = np.array([ tmax, tmax])      # Control Upper Bounds
        self.u_lb = np.array([-tmax,-tmax])      # Control Lower Bounds
        
        # Default State and inputs        
        self.xbar = np.zeros(n)
        self.ubar = np.array([0,0])
        
        self.setparams()
        
        # Ploting param
        self.n_pts        = 3 # number of pts to plot on the manipulator 
        self.dim_pts      = 2 # number of dimension for each pts 
        self.axis_to_plot = [0,1]
        
        
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        self.l1  = 1 
        self.l2  = 1
        self.lc1 = 1
        self.lc2 = 1
        
        self.m1 = 1
        self.I1 = 1
        self.m2 = 1
        self.I2 = 1
        
        self.g = 9.81
        
        self.d1 = 1
        self.d2 = 1
        
        # Total length
        self.lw  = (self.l1+self.l2)
        
        
    ##############################
    def trig(self, q = np.zeros(2)):
        """ 
        Compute cos and sin usefull in other computation 
        ------------------------------------------------
        
        """
        
        c1  = np.cos( q[0] )
        s1  = np.sin( q[0] )
        c2  = np.cos( q[1] )
        s2  = np.sin( q[1] )
        c12 = np.cos( q[0] + q[1] )
        s12 = np.sin( q[0] + q[1] )
        
        return [c1,s1,c2,s2,c12,s12]
        
        
    ##############################
    def fwd_kinematic(self, q = np.zeros(2)):
        """ 
        Compute p = [x;y] positions given angles q 
        ----------------------------------------------------
        - points of interest for ploting
        - last point should be the end-effector
        
        """
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        PTS = np.zeros(( self.n_pts , self.dim_pts ))
        
        PTS[0,0] = 0
        PTS[0,1] = 0
        
        PTS[1,0] = self.l1 * s1
        PTS[1,1] = self.l1 * c1
        
        PTS[2,0] = self.l1 * s1 + self.l2 * s12
        PTS[2,1] = self.l1 * c1 + self.l2 * c12
                
        return PTS
    
    
    ##############################
    def jacobian_endeffector(self, q = np.zeros(2)):
        """ 
        Compute jacobian of end-effector 
        --------------------------------------

        # Differential kinematic
        p : end-effector position
        q : joint coordinates
        dp = [ J_end ] dq 
        
        # Virtual work
        f_ext : force applied on end-effector
        f_g   : generalized forces in joint coordinates
        f_q = [ J_end ]^(T) f_ext
        
        """
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        J = np.zeros((2,2))
        
        J[0,0] =  self.l1 * c1 + self.l2 * c12
        J[1,0] = -self.l1 * s1 - self.l2 * s12
        J[0,1] =  self.l2 * c12
        J[1,1] = -self.l2 * s12
        
        return J
        
        
    ##############################
    def jacobian_actuators(self, q = np.zeros(2) ):
        """ 
        Compute jacobian of acutator coordinates 
        ----------------------------------------
        dim( J_a ) = ( dof , dof )
        
        # Differential kinematic
        a : actuator coordinates
        q : joint coordinates
        da = [ J_a ] dq 
        
        # Virtual work
        e     : actuator efforts
        f_g   : generalized forces in joint coordinates
        f_q = [ J_end ]^(T) e
        
        Note : This will be identity matrix for most case where actuators 
        are coupled 1:1 at each joint.
        
        """
        
        J_a = np.eye( self.dof )  # By default, identity matrix --> actuator coord = joint coord
        
        return J_a
        
        
    ##############################
    def jacobian_actuators_diff(self, q = np.zeros(2) , dq = np.zeros(2) ):
        """ 
        Compute time differential of actuator coordinates jacobian
        ----------------------------------------------------------
        dim( dJ_a ) = ( dof , dof )
        
        - Become necessarly in EoM computation if two condition are True:
        -- Inertia in actuator coordinates is included
        -- Actuator coupling is variable --> J_a = J_a( q )
        
        """
        
        dJ_a = np.zeros( ( self.dof , self.dof )  )  # By default, J is constant hence dJ = zero
        
        return dJ_a
        
    
    ##############################
    def a2q(self, a = np.zeros(2) ):
        """ 
        Get actuator coor from joint coor
        ----------------------------------------------
        by defaut joint coord & actuator coord are the same :
        
        q = a
        
        """
        
        q = a
        
        return q
        
    
    ##############################
    def q2a(self, q = np.zeros(2) ):
        """ 
        Get joint coord from actuators coor
        ----------------------------------------
        by defaut joint coord & actuator coord are the same :
        
        q = a
        
        """
        
        a = q
        
        return a
        
        
    ##############################
    def H(self, q = np.zeros(2)):
        """ 
        Inertia matrix of the manipulator
        ----------------------------------
        dim( H ) = ( dof , dof )
        
        such that --> Kinetic Energy = 0.5 * dq^T * H(q) * dq
        
        """  
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        H = np.zeros((2,2))
        
        H[0,0] = self.m1 * self.lc1**2 + self.I1 + self.m2 * ( self.l1**2 + self.lc2**2 + 2 * self.l1 * self.lc2 * c2 ) + self.I2
        H[1,0] = self.m2 * self.lc2**2 + self.m2 * self.l1 * self.lc2 * c2 + self.I2
        H[0,1] = H[1,0]
        H[1,1] = self.m2 * self.lc2 ** 2 + self.I2
        
        return H
        
        
    ##############################
    def C(self, q = np.zeros(2) ,  dq = np.zeros(2) ):
        """ 
        Corriolis and Centrifugal Matrix 
        ------------------------------------
        dim( C ) = ( dof , dof )
        
        such that --> d H / dt =  C + C^T
        
        
        """  
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        h = self.m2 * self.l1 * self.lc2 * s2
        
        C = np.zeros((2,2))
        
        C[0,0] = - h  * dq[1]
        C[1,0] =   h  * dq[0]
        C[0,1] = - h * ( dq[0] + dq[1] )
        C[1,1] = 0
        
        return C
        
        
    ##############################
    def D(self, q = np.zeros(2) ,  dq = np.zeros(2) ):
        """ 
        Damping Matrix  
        -------------------------------
        dim( D ) = ( dof , dof )
        
        f_d = damping force in joint coord
        
        f_d = D( q , dq ) * dq
        
        
        """  
               
        D = np.zeros((2,2))
        
        D[0,0] = self.d1
        D[1,0] = 0
        D[0,1] = 0
        D[1,1] = self.d2
        
        return D
        
        
    ##############################
    def G(self, q = np.zeros(2) ):
        """
        Gravity forces 
        ---------------------------
        dim( G ) = ( dof )
        
        compute gravitational force in joint coordinates       
        
        
        """  
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        g1 = (self.m1 * self.lc1 + self.m2 * self.l1 ) * self.g
        g2 = self.m2 * self.lc2 * self.g
        
        G = np.zeros(2)
        
        G[0] = - g1 * s1 - g2 * s12
        G[1] = - g2 * s12
        
        return G
        
        
    ##############################
    def F_ext(self, q = np.zeros(2) , dq = np.zeros(2) ):
        """
        Compute External forces applied on end-effector
        ---------------------------
        Override this function with a specific function if the robot
        is interacting with an external system and it will be automatically
        included in EoM that is calling this function.
        
        F_ext = f( q , dq )
                
        """  
        
        # TODO should define a dimension corresponding to end-effector DoF that can be different from the robot DoF
        F_ext = np.zeros( self.dof ) 
        
        return F_ext
        
        
    ##############################
    def e_potential(self, q = np.zeros(2) ):
        """ Compute potential energy of manipulator """  
        
        [c1,s1,c2,s2,c12,s12] = self.trig( q )
        
        g1 = (self.m1 * self.lc1 + self.m2 * self.l1 ) * self.g
        g2 = self.m2 * self.lc2 * self.g
        
        e_p = g1 * c1 + g2 * c12        
        
        return e_p
        
        
    ##########################################################################
    # Based functions : No need to modify the next functions for custom robots
    ##########################################################################
        
    ##############################
    def da2dq(self, da = np.zeros(2) , q = np.zeros(2) ):
        """ 
        Get actuator velocities from joint velocities
        ----------------------------------------------
        
        da = [ J_a( q ) ] dq
        
        """
        
        J_a = self.jacobian_actuators( q )
        
        dq  = np.dot( np.linalg.inv( J_a ) , da )
        
        return dq
        
    
    ##############################
    def dq2da(self, dq = np.zeros(2) , q = np.zeros(2) ):
        """ 
        Get joint velocities from actuators velocities
        ----------------------------------------
        
        da = [ J_a( q ) ] dq
        
        """
        
        J_a = self.jacobian_actuators( q )
        
        da  = np.dot( J_a , dq )
        
        return da
        
    
    ##############################
    def B(self, q = np.zeros(2) ):
        """
        Actuator mechanical advantage Matrix
        ---------------------------
        dim( B ) = ( dof , dof )  --> assuming number of actuator == number of DOF
        
        e   : actuator efforts
        f   : generalized force in joint coord
        
        f  = B( q ) * e
    

        """  
        
        B = self.jacobian_actuators( q ).T
        
        return B
        
    
    ##############################
    def F(self, q = np.zeros(2) , dq = np.zeros(2) , ddq = np.zeros(2)):
        """ Computed torques given a trajectory (inverse dynamic) """  
        
        H   = self.H( q )
        C   = self.C( q , dq )
        D   = self.D( q , dq )
        G   = self.G( q )
        B   = self.B( q )
        
        # External forces
        J_e = self.jacobian_endeffector( q )
        f_e = self.F_ext( q , dq )
        
        # Generalized forces
        F = np.dot( H , ddq ) + np.dot( C , dq ) + np.dot( D , dq ) + G - np.dot( J_e.T , f_e )
        
        # Actuator effort
        e = np.dot( np.linalg.inv( B ) , F )
        
        return e
        
        
    ##############################
    def ddq(self, q = np.zeros(2) , dq = np.zeros(2) , e = np.zeros(2)):
        """ Computed accelerations given torques"""  
        
        H = self.H( q )
        C = self.C( q , dq )
        D = self.D( q , dq )
        G = self.G( q )
        B = self.B( q )
        
        # External forces
        J_e = self.jacobian_endeffector( q )
        f_e = self.F_ext( q , dq )
        
        ddq = np.dot( np.linalg.inv( H ) ,  ( np.dot( B , e ) + np.dot( J_e.T , f_e ) - np.dot( C , dq ) - np.dot( D , dq ) - G ) )
        
        return ddq
        
        
    #############################
    def fc(self, x = np.zeros(4) , u = np.zeros(2) , t = 0 ):
        """ 
        Continuous time function evaluation
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        [ q , dq ] = self.x2q( x )   # from state vector (x) to angle and speeds (q,dq)
        
        ddq = self.ddq( q , dq , u ) # compute state derivative 
        
        dx = self.q2x( dq , ddq )    # from angle and speeds diff (dq,ddq) to state vector diff (dx)
        
        return dx
        
        
    #############################
    def x2q( self, x = np.zeros(4) ):
        """ from state vector (x) to angle and speeds (q,dq) """
        
        q  = x[ 0        : self.dof ]
        dq = x[ self.dof : self.n   ]
        
        return [ q , dq ]
        
        
    #############################
    def q2x( self, q = np.zeros(2) , dq = np.zeros(2) ):
        """ from angle and speeds (q,dq) to state vector (x) """
        
        x = np.zeros( self.n )
        
        x[ 0        : self.dof ] = q
        x[ self.dof : self.n   ] = dq
        
        return x
        
        
    ##############################
    def e_kinetic(self, q = np.zeros(2) , dq = np.zeros(2) ):
        """ Compute kinetic energy of manipulator """  
        
        e_k = 0.5 * np.dot( dq , np.dot( self.H( q ) , dq ) )
        
        return e_k
        
    
    ##############################
    def energy_values(self, x = np.zeros(4)  ):
        """ Compute energy values of manipulator """ 
        
        [ q , dq ] = self.x2q( x )   # from state vector (x) to angle and speeds (q,dq)
        
        e_k = self.e_kinetic( q , dq )
        e_p = self.e_potential( q )
        e   = e_k + e_p
        
        return [ e , e_k , e_p ]
        
        
    #############################
    def show(self, q = np.zeros(2) ):
        """ Plot figure of configuration q """
        
        fig = plt.figure()
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-self.lw, self.lw), ylim=(-self.lw, self.lw))
        ax.grid()
        
        pts = self.fwd_kinematic( q )
        
        line = ax.plot( pts[:, self.axis_to_plot[0] ], pts[:, self.axis_to_plot[1] ], 'o-', self.lw )
        
        plt.show()
        
        self.fig_show  = fig
        self.line_show = line[0]
        self.ax_show   = ax
        
        #return fig , ax, line
    
    #############################
    def update_show(self, q):
        """ update figure of configuration q """
        
        pts = self.fwd_kinematic( q )
        
        self.line_show.set_data( pts[:, self.axis_to_plot[0] ], pts[:, self.axis_to_plot[1] ])
        
        
    #############################
    def show_jaco(self, q ):
        """ Plot figure of configuration q """
        
        self.show_matrix( self.jacobian_endeffector(q)  , q )
        
        
    #############################
    def show_matrix(self, J , q , plotvector = False ):
        """ Plot figure of configuration q """
        
        # Plot manipulator
        fig = plt.figure()
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
        ax.grid()
        
        pts = self.fwd_kinematic( q )
        
        line = ax.plot( pts[:,0], pts[:,1], 'o-', lw=(self.l1+self.l2) )
        
        u,s,v = np.linalg.svd( J )
        
        theta = np.arctan2( u[1,0] , u[0,0] ) * 180/3.1416
        
        V1 = u[:,0] * s[0]
        V2 = u[:,1] * s[1]
        
        #print theta,V1,V2,s[0],s[1]
        if plotvector:
            line1 = ax.plot( (pts[2,0],pts[2,0]+V1[0]), (pts[2,1],pts[2,1]+V1[1]), '-' )
            line2 = ax.plot( (pts[2,0],pts[2,0]+V2[0]), (pts[2,1],pts[2,1]+V2[1]), '-' )
        
        e = matplotlib.patches.Ellipse(xy=(pts[2,0],pts[2,1]), width=s[0]+0.01, height=s[1]+0.01, angle=theta)
        ax.add_artist(e)
        e.set_clip_box(ax.bbox)
        e.set_alpha(0.1)
        e.set_facecolor('b')
        
        
        plt.show()
        
        return fig , ax, line
        
        
        
    #############################
    def computeSim(self, x0 = np.array([0,0,0,0]) , tf = 10 , n = 1001 , solver = 'ode' ):
        """ Simulate the robot """
        
        
        self.Sim    = RDDS.Simulation( self , tf , n , solver )
        self.Sim.x0 = x0
        
        # Integrate EoM
        self.Sim.compute()
        
        
    ##############################
    def animateSim(self, time_factor_video =  1.0 , save = False , file_name = 'RobotSim' ):
        """ 
        Show Animation of the simulation 
        ----------------------------------
        time_factor_video < 0 --> Slow motion video        
        
        """  
        
        # Compensate for slightly slower ploting than it should:
        time_factor_video = time_factor_video * 1.15
        
        # Compute pts localization
        self.PTS = np.zeros(( self.n_pts , self.dim_pts , self.Sim.n ))
        
        for i in xrange( self.Sim.n ):
            
            [ q , dq ]      = self.x2q(  self.Sim.x_sol_CL[i,:]  )
            self.PTS[:,:,i] = self.fwd_kinematic( q )  # Forward kinematic
            
        # Figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
        self.ax.grid()
        
        self.line, = self.ax.plot([], [], 'o-', lw=2)
        self.time_template = 'time = %.1fs'
        self.time_text = self.ax.text(0.05, 0.9, '', transform=self.ax.transAxes)
            
        inter      =  40.             # ms --> 25 frame per second
        frame_dt   =  inter / 1000. 
        
        if ( frame_dt * time_factor_video )  < self.Sim.dt :
            # Simulation is slower than video
            self.skip_steps = 1                                         # don't skip steps
            inter           = self.Sim.dt * 1000. / time_factor_video   # adjust frame speed to simulation
            n_frame         = self.Sim.n
            
        else:
            # Simulation is faster than video
            self.skip_steps =  int( frame_dt / self.Sim.dt * time_factor_video ) # --> number of simulation frame to skip between video frames
            n_frame         =  int( self.Sim.n / self.skip_steps )               # --> number of video frames
        
        # ANIMATION
        # blit=True option crash on mac
        #self.ani = animation.FuncAnimation( self.fig, self.__animate__, n_frame , interval = inter, blit=True, init_func=self.__ani_init__)
        self.ani = animation.FuncAnimation( self.fig, self.__animate__, n_frame , interval = inter , init_func=self.__ani_init__)
        
        if save:
            self.ani.save( file_name + '.mp4' ) # , writer = 'mencoder' )
            
        
        
    
    #############################
    def plotAnimation(self, x0 = np.array([0,0,0,0]) , tf = 10 , n = 1001 , solver = 'ode' ,  save = False , file_name = 'RobotSim'  ):
        """ Simulate and animate robot """
        
        self.computeSim( x0 , tf , n , solver )
        
        self.animateSim( 1.0 , save , file_name )
        
        
        
    def __ani_init__(self):
        self.line.set_data([], [])
        self.time_text.set_text('')
        return self.line, self.time_text
    
    
    def __animate__(self,i):
        thisx = self.PTS[:, self.axis_to_plot[0] , i * self.skip_steps ]
        thisy = self.PTS[:, self.axis_to_plot[1] , i * self.skip_steps ]
    
        self.line.set_data(thisx, thisy)
        self.time_text.set_text(self.time_template % ( i * self.skip_steps * self.Sim.dt ))
        return self.line, self.time_text
        
    def __animateStop__(self,i):
        
        if i > 198: # Hack To close window used in reinforcement learning algo
            plt.close()
        thisx = self.PTS[:, self.axis_to_plot[0] , i * self.skip_steps ]
        thisy = self.PTS[:, self.axis_to_plot[1] , i * self.skip_steps ]
    
        self.line.set_data(thisx, thisy)
        self.time_text.set_text(self.time_template % ( i * self.skip_steps * self.Sim.dt ))
        return self.line, self.time_text

            
'''
#################################################################
##################       1DOF Manipulator Class          ########
#################################################################
'''


class OneLinkManipulator( TwoLinkManipulator ) :
    """ 
    1DOF Manipulator Class
    -----------------------------------
    Pendulum used as exemple
    
    some base function are overloaded with the scalar version when necessarly
    -- for instance function using np.linalg.inv 
    
    """
    
    
    ############################
    def __init__(self, n = 2 , m = 1 ):
        
        RDDS.DynamicSystem.__init__(self, n , m )
        
        self.dof = 1 # Number of DoF
        
        self.state_label = ['Angle 1','Speed 1']
        self.input_label = ['Torque 1']
        
        self.state_units = ['[rad]','[rad/sec]']
        self.input_units = ['[Nm]']
        
        self.x_ub = np.array([ 2*np.pi , 2*np.pi])    # States Upper Bounds
        self.x_lb = np.array([-2*np.pi,-2*np.pi])    # States Lower Bounds
        
        tmax = 10
        
        self.u_ub = np.array([ tmax])      # Control Upper Bounds
        self.u_lb = np.array([-tmax])      # Control Lower Bounds
        
        # Default State and inputs        
        self.xbar = np.zeros(n)
        self.ubar = np.array([0])
        
        self.setparams()
        
        # Ploting param
        self.n_pts        = 2 # number of pts to plot on the manipulator 
        self.dim_pts      = 2 # number of dimension for each pts 
        self.axis_to_plot = [0,1]
        
        
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        self.l1  = 1 
        self.lc1 = 1
        
        self.m1 = 1
        self.I1 = 1
        
        # load
        self.M  = 1
        
        self.g = 9.81
        
        self.d1 = 0
        
        # Total length
        self.lw = 1
        
        
    ##############################
    def trig(self, q ):
        """ Compute cos and sin """
        
        c1  = np.cos( q )
        s1  = np.sin( q )

        
        return [c1,s1]
        
        
    ##############################
    def fwd_kinematic(self, q = np.zeros(1) ):
        """ 
        Compute p = [x;y] positions given angles q 
        ----------------------------------------------------
        - points of interest for ploting
        - last point should be the end-effector
        
        """
        
        [c1,s1] = self.trig( q )
        
        PTS = np.zeros(( self.n_pts , self.dim_pts ))
        
        PTS[0,0] = 0
        PTS[0,1] = 0
        
        PTS[1,0] = self.l1 * s1
        PTS[1,1] = self.l1 * c1
        
        return PTS
    
    
    ##############################
    def jacobian_endeffector(self, q = np.zeros(1)):
        """ 
        Compute jacobian of end-effector 
        --------------------------------------
        
        # Differential kinematic
        p : end-effector position
        q : joint coordinates
        dp = [ J_end ] dq 
        
        # Virtual work
        f_ext : force applied on end-effector
        f_g   : generalized forces in joint coordinates
        f_q = [ J_end ]^(T) f_ext
        
        """
        
        [c1,s1] = self.trig( q )
        
        J = np.zeros((2,1))
        
        J[0] =  self.l1 * c1 
        J[1] = -self.l1 * s1 
        
        return J
        
        
    ##############################
    def jacobian_actuators(self, q = np.zeros(2) ):
        """ 
        Compute jacobian of acutator coordinates 
        ----------------------------------------
        dim( J_a ) = ( dof , dof )
        
        # Differential kinematic
        a : actuator coordinates
        q : joint coordinates
        da = [ J_a ] dq 
        
        # Virtual work
        e     : actuator efforts
        f_g   : generalized forces in joint coordinates
        f_q = [ J_end ]^(T) e
        
        Note : This will be identity matrix for most case where actuators 
        are coupled 1:1 at each joint.
        
        """
        
        J_a = 1  # By default, identity matrix --> actuator coord = joint coord
        
        return J_a
        
        
    ##############################
    def jacobian_actuators_diff(self, q = np.zeros(2) , dq = np.zeros(2) ):
        """ 
        Compute time differential of actuator coordinates jacobian
        ----------------------------------------------------------
        dim( dJ_a ) = ( dof , dof )
        
        - Become necessarly in EoM computation if two condition are True:
        -- Inertia in actuator coordinates is included
        -- Actuator coupling is variable --> J_a = J_a( q )
        
        """
        
        dJ_a = 0  # By default, J is constant hence dJ = zero
        
        return dJ_a
        
        
    ##############################
    def H(self, q = np.zeros(1)):
        """ Inertia matrix """  
        
        H = self.m1 * self.lc1**2 + self.I1 + self.M * ( self.l1**2 )
        
        return H
        
        
    ##############################
    def C(self, q = np.zeros(1) ,  dq = np.zeros(1) ):
        """ Corriolis Matrix """  
        
        C = 0

        
        return C
        
        
    ##############################
    def D(self, q = np.zeros(1) ,  dq = np.zeros(1) ):
        """ Damping Matrix """  
               
        D = 0
        
        return D
        
        
    ##############################
    def G(self, q = np.zeros(1) ):
        """Gravity forces """  
        
        [c1,s1] = self.trig( q )
        
        g1 = (self.m1 * self.lc1 + self.M * self.l1 ) * self.g
        
        G = - g1 * s1 
        
        return G
        
    
    ##############################
    def B(self, q = np.zeros(2) ):
        """
        Actuator mechanical advantage Matrix
        ---------------------------
        dim( B ) = ( dof , dof )  --> assuming number of actuator == number of DOF
        
        e   : actuator efforts
        f   : generalized force in joint coord
        
        f  = B( q ) * e
    

        """  
        
        B = self.jacobian_actuators( q )
        
        return B
        
        
    
    ##############################
    def F(self, q = np.zeros(1) , dq = np.zeros(1) , ddq = np.zeros(1)):
        """ Computed torques given a trajectory (inverse dynamic) """  
        
        H = self.H( q )
        C = self.C( q , dq )
        D = self.D( q , dq )
        G = self.G( q )
        B = self.B( q )
        
        # Generalized forces
        F = np.dot( H , ddq ) + np.dot( C , dq ) + np.dot( D , dq ) + G
        
        # Actuator effort
        e = ( 1./ B ) * F 
        
        return e
        
        
    ##############################
    def ddq(self, q = np.zeros(1) , dq = np.zeros(1) , e = np.zeros(1) ):
        """ Computed accelerations given torques"""  
        
        H = self.H( q )
        C = self.C( q , dq )
        D = self.D( q , dq )
        G = self.G( q )
        B = self.B( q )
        
        ddq = np.dot( 1./H ,  ( np.dot( B , e ) - np.dot( C , dq ) - np.dot( D , dq ) - G  ) )
        
        #  TODO include external forces for 1-DOF
        
        return ddq
                
        
    #############################
    def x2q( self, x = np.zeros(2) ):
        """ from state vector (x) to angle and speeds (q,dq) """
        
        q  = x[ 0 ]
        dq = x[ 1 ]
        
        return [ q , dq ]
        
        
    #############################
    def q2x( self, q = np.zeros(1) , dq = np.zeros(1) ):
        """ from angle and speeds (q,dq) to state vector (x) """
        
        x = np.zeros( self.n )
        
        x[ 0  ] = q
        x[ 1  ] = dq
        
        return x
        
        
    ##############################
    def e_kinetic(self, q = np.zeros(1) , dq = np.zeros(1) ):
        """ Compute kinetic energy of manipulator """  
        
        e_k = 0.5 * np.dot( dq , np.dot( self.H( q ) , dq ) )
        
        return e_k
        
        
    ##############################
    def e_potential(self, q = np.zeros(2) ):
        """ Compute potential energy of manipulator """  
        
        [ c1 , s1 ] = self.trig( q )
        
        g1 = (self.m1 * self.lc1 + self.M * self.l1 ) * self.g
        
        e_p = g1 * c1   
        
        return e_p
        
        
    ##############################
    def da2dq(self, da = np.zeros(1) , q = np.zeros(1) ):
        """ 
        Get actuator velocities from joint velocities
        ----------------------------------------------
        
        da = [ J_a( q ) ] dq
        
        """
        
        J_a = self.jacobian_actuators( q )
        
        dq  = np.dot( 1./ J_a  , da )
        
        return dq
        




'''
#################################################################
##################       3DOF Manipulator Class          ########
#################################################################
'''



class ThreeLinkManipulator( TwoLinkManipulator ) :
    """
    3DOF Manipulator Class 
    -------------------------------
    
    base:     revolute arround z
    shoulder: revolute arround y
    elbow:    revolute arround y
    
    see Example 4.3 in
    http://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-manipdyn_v1_2.pdf
    
    
    """
    
    
    ############################
    def __init__(self, n = 6 , m = 3 ):
        
        RDDS.DynamicSystem.__init__(self, n , m )
        
        self.dof = 3      # Number of degree of freedoms
        
        self.state_label = ['Angle 1','Angle 2','Angle 3','Speed 1','Speed 2','Speed 3']
        self.input_label = ['Torque 1','Torque 2','Torque 3']
        
        self.state_units = ['[rad]','[rad]','[rad]','[rad/sec]','[rad/sec]','[rad/sec]']
        self.input_units = ['[Nm]','[Nm]','[Nm]']
        
        self.x_ub = np.array([ 6, 6, 6, 6, 6, 6])    # States Upper Bounds
        self.x_lb = np.array([-6,-6,-6,-6,-6,-6])    # States Lower Bounds
        
        tmax = 1 # Max torque
        
        self.u_ub = np.array([ tmax, tmax, tmax])      # Control Upper Bounds
        self.u_lb = np.array([-tmax,-tmax,-tmax])      # Control Lower Bounds
        
        # Default State and inputs        
        self.xbar = np.zeros(n)
        self.ubar = np.array([0,0,0])
        
        self.setparams()
        
        # Ploting param
        self.n_pts   = 4 # number of pts to plot on the manipulator 
        self.dim_pts = 3 # number of dimension for each pts 
        self.axis_to_plot = [0,2]  # axis to plot for 2D plot
        
        
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        # Kinematic
        self.l1  = 1 
        self.l2  = 1
        self.l3  = 1
        self.lc1 = 1
        self.lc2 = 1
        self.lc3 = 1
        
        # Inertia
        self.m1 = 1
        self.m2 = 1
        self.m3 = 1
        
        self.I1z = 1
        
        self.I2x = 1
        self.I2y = 1
        self.I2z = 1
        
        self.I3x = 1
        self.I3y = 1
        self.I3z = 1
        
        # Gravity
        self.g = 9.81
        
        # Joint damping
        self.d1 = 1
        self.d2 = 1
        self.d3 = 1
        
        # Total length
        self.lw  = (self.l1+self.l2+self.l3)
        
        
    ##############################
    def trig(self, q = np.zeros(3) ):
        """ Compute cos and sin """
        
        c1  = np.cos( q[0] )
        s1  = np.sin( q[0] )
        c2  = np.cos( q[1] )
        s2  = np.sin( q[1] )
        c3  = np.cos( q[2] )
        s3  = np.sin( q[2] )
        c12 = np.cos( q[0] + q[1] )
        s12 = np.sin( q[0] + q[1] )
        c23 = np.cos( q[2] + q[1] )
        s23 = np.sin( q[2] + q[1] )
        
        return [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23]
        
        
    ##############################
    def fwd_kinematic(self, q = np.zeros(3) ):
        """ Compute [x;y;z] end effector position given angles q """
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        # Three robot points
        
        # Base of the robot
        p0 = [0,0,0]
        
        # Shperical point 
        p1 = [ 0, 0, self.l1 ]
        
        # Elbow
        z2 = self.l1 - self.l2 * s2
        
        r2 = self.l2 * c2
        x2 = r2 * c1
        y2 = r2 * s1
        
        p2 = [ x2, y2, z2 ]
        
        # End-effector
        z3 = self.l1 - self.l2 * s2 - self.l3 * s23
        
        r3 = self.l2 * c2 + self.l3 * c23
        x3 = r3 * c1
        y3 = r3 * s1
        
        p3 = [ x3, y3, z3 ]
        
        return np.array([p0,p1,p2,p3])
        
                    
    ##############################
    def jacobian_endeffector(self, q = np.zeros(3)):
        """ Compute jacobian of end-effector """
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        J = np.zeros((3,3))
        
        J[0,0] =  -( self.l2 * c2 + self.l3 * c23 ) * s1
        J[0,1] =  -( self.l2 * s2 + self.l3 * s23 ) * c1
        J[0,2] =  - self.l3 * s23 * c1
        J[1,0] =   ( self.l2 * c2 + self.l3 * c23 ) * c1
        J[1,1] =  -( self.l2 * s2 + self.l3 * s23 ) * s1
        J[1,2] =  - self.l3 * s23 * s1
        J[2,0] =  0
        J[2,1] =  -( self.l2 * c2 + self.l3 * c23 )
        J[2,2] =  - self.l3 * c23
        
        return J
        
        
    ##############################
    def H(self, q = np.zeros(3)):
        """ Inertia matrix """  
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        # variable to match the book notation
        
        m1 = self.m1
        m2 = self.m2
        m3 = self.m3
        
        Iz1 = self.I1z
        Ix2 = self.I2x
        Iy2 = self.I2y
        Iz2 = self.I2z
        Ix3 = self.I3x
        Iy3 = self.I3y
        Iz3 = self.I3z
        
        l1 = self.l2
        r1 = self.lc2
        l2 = self.l3
        r2 = self.lc3
        
        
        H = np.zeros((3,3))
        
        # TODO
        H[0,0] = Iy2 * s2 **2 + Iy3 * s23 **2 + Iz1 + Iz2 * c2 **2 + Iz3 * c23 **2 + m2 * ( r1 * c2 ) **2 + m3 * ( l1 * c2 + r2 * c23 ) **2
        H[0,1] = 0
        H[0,2] = 0
        H[1,0] = 0
        H[1,1] = Ix2 + Ix3 + m3 * l1 **2 + m2 * r1 **2 + m3 * r2 **2 + 2 * m3 *l1 * r2 * c3
        H[1,2] = Ix3 + m3 * r2 **2 + m3 * l1 * r2 * c3
        H[2,0] = 0
        H[2,1] = H[1,2]
        H[2,2] = Ix3 + m3 * r2 ** 2
        
        return H
        
        
    ##############################
    def C(self, q = np.zeros(3) ,  dq = np.zeros(3) ):
        """ Corriolis Matrix """ 
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        # variable to match the book notation
        
        m1 = self.m1
        m2 = self.m2
        m3 = self.m3
        
        Iz1 = self.I1z
        Ix2 = self.I2x
        Iy2 = self.I2y
        Iz2 = self.I2z
        Ix3 = self.I3x
        Iy3 = self.I3y
        Iz3 = self.I3z
        
        l1 = self.l2
        r1 = self.lc2
        l2 = self.l3
        r2 = self.lc3
        
        
        T112 = ( Iy2 - Iz2 - m2 * r1 **2 ) * c2 * s2 + ( Iy3 - Iz3 ) * c23 * s23 - m3 * ( l1 * c2 + r2 * c23 ) * ( l1 * s2 + r2 * s23 )
        T113 = ( Iy3 - Iz3 ) * c23 * s23 - m3 * r2 * s23 * ( l1 * c2 + r2 * c23 )
        T121 = T112
        T131 = T113
        T211 = ( Iz2 - Iy2 + m2 * r1 **2 ) * c2 * s2 + ( Iz3 - Iy3 ) * c23 * s23 + m3 * ( l1 * c2 + r2 * c23 ) * ( l1 * s2 + r2 * s23 )
        T223 = - l1 * m3 * r2 * s3
        T232 = T223
        T233 = T223
        T311 = ( Iz3 - Iy3 ) * c23 * s23 + m3 * r2 * s23 * ( l1 * c2 + r2 * c23 )
        T322 = l1 * m3 * r2 * s3
                
        C = np.zeros((3,3))
        
        C[0,0] = T112 * dq[1] + T113 * dq[2]
        C[0,1] = T121 * dq[0]
        C[0,2] = T131 * dq[0]
        
        C[1,0] = T211 * dq[0]
        C[1,1] = T223 * dq[2]
        C[1,2] = T232 * dq[1] + T233 * dq[2]
        
        C[2,0] = T311 * dq[0]
        C[2,1] = T322 * dq[1]
        C[2,2] = 0 
        
        return C
        
        
    ##############################
    def D(self, q = np.zeros(3) ,  dq = np.zeros(3) ):
        """ Damping Matrix """  
               
        D = np.zeros((3,3))
        
        D[0,0] = self.d1
        D[1,1] = self.d2
        D[2,2] = self.d3
        
        return D
        
        
    ##############################
    def G(self, q = np.zeros(2) ):
        """Gravity forces """  
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        G = np.zeros(3)
        
        G[0] = 0
        G[1] = -(self.m2 * self.g * self.lc2 + self.m3 * self.g * self.l2 ) * c2 - self.m3 * self.g * self.lc3 * c23
        G[2] = - self.m3 * self.g * self.lc3 * c23
        
        return G
        
        
    ##############################
    def e_potential(self, q = np.zeros(2) ):
        """ Compute potential energy of manipulator """  
               
        e_p = 0
        
        # TODO
        
        return e_p
        
        
    ###############################
    def show_3D(self, q ):
        """ Plot figure of configuration q """
        
        pts = self.fwd_kinematic( q )
                
        #Import matplotlib as mpl
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        line = ax.plot( pts[:,0], pts[:,1], pts[:,2], 'o-' )   
        # Setting the axes properties
        ax.set_xlim3d([ - self.lw / 2. , self.lw / 2. ])
        ax.set_xlabel('X')
        ax.set_ylim3d([- self.lw / 2. , self.lw / 2.])
        ax.set_ylabel('Y')
        ax.set_zlim3d([- self.lw / 2. , self.lw / 2.])
        ax.set_zlabel('Z')
        plt.show()
        
        
    ##############################
    def animate3DSim(self, time_factor_video =  1.0 , save = False , file_name = 'RobotSim' ):
        """ 
        Show Animation of the simulation 
        ----------------------------------
        self.computeSim() need to be run before
        
        time_factor_video < 0 --> Slow motion video        
        
        """  
        
        # Compensate for slightly slower ploting than it should:
        time_factor_video = time_factor_video * 1.15
        
        # Compute pts localization
        self.PTS = np.zeros(( self.n_pts , self.dim_pts , self.Sim.n ))
        
        for i in xrange( self.Sim.n ):
            
            [ q , dq ]      = self.x2q(  self.Sim.x_sol_CL[i,:]  )
            self.PTS[:,:,i] = self.fwd_kinematic( q )  # Forward kinematic
            
        #Import matplotlib as mpl
        import mpl_toolkits.mplot3d.axes3d as p3

        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)
        self.line = self.ax.plot( self.PTS[:,0,0], self.PTS[:,1,0], self.PTS[:,2,0], 'o-' )[0]
        self.time_template = 'time = %.1fs'
        self.time_text = self.ax.text(0, 0, 0, '', transform=self.ax.transAxes)
        
        # Setting the axes properties
        self.ax.set_xlim3d([ - self.lw / 2. , self.lw / 2. ])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([- self.lw / 2. , self.lw / 2.])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([- self.lw / 2. , self.lw / 2.])
        self.ax.set_zlabel('Z')
        self.ax.set_title('3D Robot Animation')
        
        inter      =  40.             # ms --> 25 frame per second
        frame_dt   =  inter / 1000. 
        
        if ( frame_dt * time_factor_video ) < self.Sim.dt :
            # Simulation is slower than video
            self.skip_steps = 1                                         # don't skip steps
            inter           = self.Sim.dt * 1000. / time_factor_video   # adjust frame speed to simulation
            n_frame         = self.Sim.n
            
        else:
            # Simulation is faster than video
            self.skip_steps =  int( frame_dt / self.Sim.dt * time_factor_video ) # --> number of simulation frame to skip between video frames
            n_frame         =  int( self.Sim.n / self.skip_steps )               # --> number of video frames
        
        
        self.ani = animation.FuncAnimation( self.fig, self.__animate_3D__, n_frame, interval=inter, blit=False )
                    
        
        if save:
            self.ani.save( file_name + '.mp4' ) #, writer = 'mencoder' )
        
        
        
    #############################
    def plot3DAnimation(self, x0 = np.array([0,0,0,0,0,0]) , tf = 10 , n = 1001 ,  solver = 'ode' , save = False , file_name = 'RobotSim' ):
        """ Simulate and animate robot """
        
        self.computeSim( x0 , tf , n , solver )
        
        self.animate3DSim( 1.0 , save , file_name )
        
        
    ##############################
    def __animate_3D__(self,i):
        thisx = self.PTS[:,0, i * self.skip_steps]
        thisy = self.PTS[:,1, i * self.skip_steps]
        thisz = self.PTS[:,2, i * self.skip_steps]
    
        #self.line.set_data(thisx, thisy, thisz) # not working for 3D
        self.line.set_data(thisx, thisy)
        self.line.set_3d_properties(thisz)
        
        self.time_text.set_text(self.time_template % (i * self.skip_steps * self.Sim.dt))
        return self.line, self.time_text
        


'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    R1 = OneLinkManipulator()
    R2 = TwoLinkManipulator()
    R3 = ThreeLinkManipulator()

            
