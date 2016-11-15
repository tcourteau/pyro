# -*- coding: utf-8 -*-
"""
Created on Fri Aug 12 10:10:34 2016

@author: agirard
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d



class BoeingArm( HM.HybridThreeLinkManipulator ) :
    """ 3DOF Manipulator Class """
    
    
    ############################
    def __init__(self, n = 6 , m = 4 ):
        
        HM.HybridThreeLinkManipulator.__init__(self, n , m )
        
        # Ploting param
        self.n_pts   = 10 # number of pts to plot on the manipulator 
        self.dim_pts = 3 # number of dimension for each pts 
        self.axis_to_plot = [0,2]  # axis to plot for 2D plot
        
        # Create interpol function for the 4-bar fwd kinematic
        self.compute_a0_fwd_kinematic()
        
    #############################
    def setparams(self):
        
        self.setKineticParams()
        self.setDynamicParams()
        self.setActuatorParams()
        
        print 'Loaded Boeing Arm Manipulator !'
        
        
    #############################
    def setKineticParams(self):
        """ Set kinetic parameters here """
        
        # First link kinematic
        b0  = 0.8   # 2x2 tubing length
        b1  = 0.12  # pivot distance from 2x2 tubing base
        b2  = 0.11  # pivot height from 2x2 tubing mid-line
        b3  = 0.32  # rod length
        b4  = 0.08  # pivot height from ballscrew line
        b5  = 0.49  # total length of ballscrew
        b6  = np.sqrt( b1 **2 + b2 **2 )  # distance between pivots on main tube
        b7  = np.arctan( b2 / b1 )         # angle difference between 
        self.b = np.array([ b0 , b1 , b2 , b3 , b4 , b5 , b6 , b7])
        
        
        # Second link
        self.l1 = 0.3  # Length of link 1
        
        # Third Link
        self.l2 = 0.3
        
        
        self.lw = 1.3
        
    #############################
    def setDynamicParams(self):
        """ Set dynamic parameters here """

        self.g = 9.8
        
        # Dynamics first link
        self.mc0 = 0.2              # 200g carbon tube alone
        self.lc0 = self.b[0] * 0.5
        self.Ic0 = 0

        # joint viscous damping coef
        self.d1 = 0.01 
        
        
        
    #############################
    def setActuatorParams(self ):
        """ Set actuators parameters here """
        
        # Actuator damping coef
        self.Da = np.diag( [ 0.00002 , 0.00002 , 0.00002 ] )
        
        # Actuator inertia coef
        
        I_m = 15 # gcm2
        
        self.Ia = np.diag( [ I_m , I_m , I_m ] ) * 0.001 * ( 0.01 **2 )
        
        # Gear ratio
        lead = 0.02
        r1   = 4
        r2   = 72
        
        R1 = np.diag([ 2 * np.pi * r1 / lead ,1,1])
        R2 = np.diag([ 2 * np.pi * r2 / lead ,1,1])
        
        self.R = [ R1 , R2 ]
        
     
    ##############################
    def trig(self, q = np.zeros(3) ):
        """ Compute cos and sin """
        
        theta_0 =  q[0]  # non-linear 4-bar linkage
        s0 = np.sin( theta_0 )
        c0 = np.cos( theta_0 )
        
        theta_3 = self.b[7] - theta_0
        s3 = np.sin( theta_3 )
        c3 = np.cos( theta_3 )
        
        theta_2 = np.arcsin( (self.b[6] * c3 - self.b[4] ) / self.b[3] )
        s2 = np.sin( theta_2 )
        c2 = np.cos( theta_2 )
        
        base  = [ theta_0 , s0 , c0 , theta_2 , s2 , c2 , theta_3, s3 , c3 ]
        
        theta_1 = q[1]
        s1      = np.sin( theta_1 )
        c1      = np.cos( theta_1 )
        s01     = np.sin( theta_0 + theta_1 )
        c01     = np.cos( theta_0 + theta_1 )
        
        link1 = [ theta_1 , s1 , c1 , s01 , c01 ]
        
        theta_2 = q[2]
        s2      = np.sin( theta_2 )
        c2      = np.cos( theta_2 )
        s012    = np.sin( theta_0 + theta_1 + theta_2 )
        c012    = np.cos( theta_0 + theta_1 + theta_2 )
        
        link2 = [ theta_2 , s2 , c2 , s012 , c012 ]
        
        
        return [base,link1,link2]
        
        
    ##############################
    def fwd_kinematic(self, q = np.zeros(3) ):
        """ Compute [x;y;z] positions of points of interest given angles q """
        
        ### First DOF ### 
        [base,link1,link2] = self.trig( q )
        
        a_0 = self.a0_inv_kinematic( q[0] ) # actuator length
        
        p0 = [ -self.b[5] , 0 , 0 ]
        p1 = [ 0, 0, 0 ]                      # Base of 2x2 beam
        p2 = [ a_0 - self.b[5] , 0 , 0 ]
        p3 = [ a_0 - self.b[5] , 0 , self.b[4] ]
        
        [ theta_0 , s0 , c0 , theta_2 , s2 , c2 , theta_3, s3 , c3 ] = base
        
        p4 = [ a_0 - self.b[5] + self.b[3] * c2 , 0 , self.b[4] + self.b[3] * s2 ]
        p5 = [ self.b[1] * s0 , 0 , self.b[1] * c0 ]
        p6 = [ 0 , 0 , 0 ]
        p7 = [ self.b[0] * s0 , 0 , self.b[0] * c0 ]
        
        ### Second DOF ###
        
        [ theta_1 , s1 , c1 , s01 , c01 ] = link1
        
        p8 = [ self.l1 * s01 + p7[0] , 0 , self.l1 * c01 + p7[2] ]
        
        
        ### Third DOF ###
        
        [ theta_2 , s2 , c2 , s012 , c012 ] = link2
        
        p9 = [ self.l2 * s012 + p8[0] , 0 , self.l2 * c012 + p8[2] ]
        
        
        return np.array([p0,p1,p2,p3,p4,p5,p6,p7,p8,p9])
        
                    
    ##############################
    def jacobian_endeffector(self, q = np.zeros(3)):
        """ Compute jacobian of end-effector """
                
        J = np.eye( self.dof )
        #TODO
        
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
        
        J_a[0,0] = self.jacobian_a0_q0( q ) # non-linear 4-bar linkage
        
        return J_a
        
    
    ##############################
    def a2q(self, a = np.zeros(3) ):
        """ 
        Get actuator coor from joint coor
        ----------------------------------------------

        """
        
        q = np.zeros( self.dof )
        
        q[0] = self.q0_fwd_kinematic( a[0] )  # Four bar kinematic
        q[1] = a[1]
        q[2] = a[2]
        
        
        return q
        
    
    ##############################
    def q2a(self, q = np.zeros(3) ):
        """ 
        Get joint coord from actuators coor
        ----------------------------------------
        
        """
        
        a = np.zeros( self.dof )
        
        a[0]  = self.a0_inv_kinematic( q[0] ) # Four bar kinematic
        a[1]  = q[1]                          # joint revolute actuator
        a[2]  = q[2]                          # joint revolute actuator
        
        return a
         
        
    ##############################
    def H(self, q = np.zeros(3)):
        """ Inertia matrix """  
        
        H = np.eye(3)
        
        # Temp 1-DoF
        
        H[0,0] = self.mc0 * self.lc0 ** 2 + self.Ic0 
        
        # TODO        
        
        return H
        
        
    ##############################
    def C(self, q = np.zeros(3) ,  dq = np.zeros(3) ):
        """ Corriolis Matrix """ 
                        
        C = np.zeros((3,3))
        #TODO
        
        return C
        
        
    ##############################
    def D(self, q = np.zeros(3) ,  dq = np.zeros(3) ):
        """ Damping Matrix """  
               
        D = np.zeros((3,3))
        
        D[0,0] = self.d1
        #D[1,1] = self.d2
        #D[2,2] = self.d3
        
        return D
        
        
    ##############################
    def G(self, q = np.zeros(2) ):
        """Gravity forces """  
        
        [base,link1,link2] = self.trig( q )
        
        [ theta_0 , s0 , c0 , theta_2 , s2 , c2 , theta_3, s3 , c3 ] = base
        
        #TODO
        
        G = np.zeros(3)
        
        G[0] = - self.mc0 * self.g * self.lc0 * s0
        
        return G
        
        
    ##############################
    def e_potential(self, q = np.zeros(2) ):
        """ Compute potential energy of manipulator """  
               
        e_p = 0
        
        # TODO
        
        return e_p
        
    
    ################################################################
    # Utility functions specific to Boeing Arm
    ################################################################
        
        
    ############################
    def a0_inv_kinematic(self, q_0 = 0 ):
        """ 
        Inverse kinematic of first DOF 
        -----------------------------
        q_0 : angle of 2x2 tube w/ vertical [rad]
        a_0 : linear displacement of ballscew nut from zero-position
        
        """
        # see alex logbook for figure and variable def:
        
        theta_3 = self.b[7] - q_0   # angle offset
        
        s3 = np.sin( theta_3 )
        c3 = np.cos( theta_3 )
        
        l3 = self.b[6]
        l2 = self.b[3]
        h1 = self.b[4]
        
        l1 = l3 * s3 + np.sqrt( l2 **2 - ( l3 * c3 - h1) **2 )
        
        a_0 = self.b[5] - l1
        
        return a_0
        
        
    ############################
    def compute_a0_fwd_kinematic( self , plot = False  ):
        """ 
        Create and interpol function for fwd kinematic 
        -----------------------------------------------
        Data validated using solidworks sketch tools
        
        """
        
        # Create data
        angles = np.arange( - 0.2 , 2.3 , 0.01)
        linear = np.zeros( angles.size )
        
        # Inv kinematic
        for i in range( angles.size ):
            linear[i] = self.a0_inv_kinematic( angles[i] )
            
        self.q0_fwd_kinematic = interp1d( linear , angles )
        # q0 = self.q0_fwd_kinematic( a_0 )
        
        if plot:
            # For validation
            
            linear_approx = np.arange( 0.05 , 0.33 , 0.01)
            angles_approx = np.zeros( linear_approx.size )
            
            for i in range( linear_approx.size ):
                angles_approx[i] = self.q0_fwd_kinematic( linear_approx[i] )
            
            fig , plot = plt.subplots( 1 , sharex=True , figsize=(4, 3), dpi=300, frameon=True)
            fig.canvas.set_window_title('First Link kinematic')
            plt.plot( linear, angles * 180 / np.pi , 'b-')
            plt.plot( linear_approx, angles_approx * 180 / np.pi , 'r-')
            plot.set_xlabel( 'Linear displacement [meter]' , fontsize=5)
            plot.set_ylabel( 'Angle 0 [deg]' , fontsize=5)
            plot.grid(True)
            fig.tight_layout()        
    
    
    ##############################
    def jacobian_a0_q0(self, q = np.zeros(3)):
        """ 
        Compute jacobian of link0 angular velocity ( dq_0 )  vs. linear actuator velocity ( da_0 )
        
        units = [meter/rad]
        
        validated by looking at the gradient of the integral function
        
        """
        
        [base,link1,link2] = self.trig( q )
        
        [ theta_0 , s0 , c0 , theta_2 , s2 , c2 , theta_3, s3 , c3 ] = base
                
        l3 = self.b[6]
        l2 = self.b[3]
        h1 = self.b[4]
        
        alpha = l3*c3 - h1
        beta  = l2 ** 2 - alpha **2
        
        j     = l3 * c3 + alpha * l3 * s3 / np.sqrt( beta )
        
        return j

        





#################################################################################################################




class TestPendulum( HM.HybridThreeLinkManipulator ) :
    """ 3DOF Manipulator Class """
    
    
    ############################
    def __init__(self, n = 6 , m = 4 ):
        
        HM.HybridThreeLinkManipulator.__init__(self, n , m )
        
        # Ploting param
        self.n_pts   = 2 # number of pts to plot on the manipulator 
        self.dim_pts = 3 # number of dimension for each pts 
        self.axis_to_plot = [0,2]  # axis to plot for 2D plot
        
        self.ubar = np.array([0,0,0,1])
        
        self.x_ub = np.array([ + 1 * np.pi , 0 , 0 ,  15 , 0 , 0])
        self.x_lb = np.array([ - 2 * np.pi , 0 , 0 , -15 , 0 , 0])
        
        self.dq_max_HF = 0.9 # [rad/sec]
        
        
    #############################
    def isavalidinput(self , x , u):
        """ check if u is in the domain """
        
        ans = False # set to True if something is bad
        
        for i in xrange(self.m):
            ans = ans or ( u[i] < self.u_lb[i] )
            ans = ans or ( u[i] > self.u_ub[i] )
            
        # add w_max constraint
        
        # if High-force mode
        if u[3] == 1 :
            
            dq = x[3]
            
            ans = ans or ( dq >  self.dq_max_HF )
            ans = ans or ( dq < -self.dq_max_HF )

            
        return not(ans)
        
        
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        self.l1  = 0.25
        self.lc1 = 0.1
        
        self.mc1 = 0.0 # Neglect
        self.Ic1 = 0.0 # Neglect
        
        # load
        self.M  = 1.0
        
        self.g = 9.81 * 1
        
        self.d1 = 0.05
        
        # Total length
        self.lw = 0.3
        
        self.setActuatorParams()
        
    #############################
    def setActuatorParams(self ):
        """ Set actuators parameters here """
        
        # Actuator damping coef
        self.Da = np.diag( [ 0.00002 , 0.00002 , 0.00002 ] ) * 0.1
        
        # Actuator inertia coef
        
        I_m = 15 # gcm2
        
        self.Ia = np.diag( [ I_m , I_m , I_m ] ) * 0.001 * ( 0.01 **2 )
        
        # Gear ratio
        r1   = 23.2 # 5.8 * 4
        r2   = 474  # 123 * 52/18 *4/3
        
        R1 = np.diag([ r1 ,1,1])
        R2 = np.diag([ r2, 1,1])
        
        self.R = [ R1 , R2 ]
        
        
    ##############################
    def trig(self, q = np.zeros(3) ):
        """ Compute cos and sin """
        
        c1  = np.cos( q[0] )
        s1  = np.sin( q[0] )

        
        return [c1,s1]
        
    ##############################
    def fwd_kinematic(self, q = np.zeros(3) ):
        """ 
        Compute p = [x;y;z] positions given angles q 
        ----------------------------------------------------
        - points of interest for ploting
        - last point should be the end-effector
        
        """
        
        [c1,s1] = self.trig( q )
        
        PTS = np.zeros(( self.n_pts , self.dim_pts ))
        
        PTS[0,0] = 0
        PTS[0,2] = 0
        
        PTS[1,0] = self.l1 * s1
        PTS[1,2] = self.l1 * c1
        
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
        
        J = np.zeros((3,3))
        
        J[0] =  self.l1 * c1
        J[2] = -self.l1 * s1 
        
        return J
        
    
    ##############################
    def H(self, q = np.zeros(3)):
        """ Inertia matrix """  
        
        H = np.eye(3)
        
        # Temp 1-DoF
        
        H[0,0] = self.mc1 * self.lc1 ** 2 + self.M * self.l1 ** 2 + self.Ic1
        
        # TODO        
        
        return H
        
        
    ##############################
    def C(self, q = np.zeros(3) ,  dq = np.zeros(3) ):
        """ Corriolis Matrix """ 
                        
        C = np.zeros((3,3))
        
        return C
        
        
    ##############################
    def D(self, q = np.zeros(3) ,  dq = np.zeros(3) ):
        """ Damping Matrix """  
               
        D = np.zeros((3,3))
        
        D[0,0] = self.d1
        #D[1,1] = self.d2
        #D[2,2] = self.d3
        
        return D
        
        
    ##############################
    def G(self, q = np.zeros(2) ):
        """Gravity forces """  

        G = np.zeros(3)
        
        [c1,s1] = self.trig( q )
        
        g1 = (self.mc1 * self.lc1 + self.M * self.l1 ) * self.g
        
        G[0] = - g1 * s1 
        
        return G
        
        
    ##############################
    def e_potential(self, q = np.zeros(2) ):
        """ Compute potential energy of manipulator """  
               
        e_p = 0
        
        # TODO
        
        return e_p



'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    BA = BoeingArm()
    
    R  = TestPendulum()
    
    BA.plotAnimation( [0.1,1.2,-0.5,0,0.1,0.1])
    
