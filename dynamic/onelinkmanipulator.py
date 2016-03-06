# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 13:09:20 2015

@author: agirard
"""

import matplotlib.animation as animation

from DynamicSystem import *


class OneLinkManipulator( DynamicSystem ) :
    """ 2DOF Manipulator Class """
    
    
    ############################
    def __init__(self, n = 2 , m = 3 ):
        
        DynamicSystem.__init__(self, n , m )
        
        self.state_label = ['Angle 1','Speed 1']
        self.input_label = ['Torque 1','Ratio','Distrubance']
        
        self.state_units = ['[rad]','[rad/sec]']
        self.input_units = ['[Nm]','','[Nm]']
        
        self.x_ub = np.array([ 2*np.pi , 2*np.pi])    # States Upper Bounds
        self.x_lb = np.array([-2*np.pi,-2*np.pi])    # States Lower Bounds
        
        tmax = 10
        
        self.u_ub = np.array([ tmax, 10 , 10])      # Control Upper Bounds
        self.u_lb = np.array([-tmax,  0 , -10])      # Control Lower Bounds
        
        # Default State and inputs        
        self.xbar = np.zeros(n)
        self.ubar = np.array([0,1,0])
        
        self.setparams()
        
        
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
        
        
    ##############################
    def trig(self, q ):
        """ Compute cos and sin """
        
        c1  = np.cos( q )
        s1  = np.sin( q )

        
        return [c1,s1]
        
        
    ##############################
    def fwd_kinematic(self, q = np.zeros(1) ):
        """ Compute [x;y] end effector position given angles q """
        
        [c1,s1] = self.trig( q )
        
        # Three robot points
        
        x0 = 0
        y0 = 0
        
        x1 = self.l1 * s1
        y1 = self.l1 * c1
        
        return np.array([[x0,y0],[x1,y1]])
    
    
    ##############################
    def jacobian_endeffector(self, q = np.zeros(1)):
        """ Compute jacobian of end-effector """
        
        [c1,s1] = self.trig( q )
        
        J = np.zeros((2,1))
        
        J[0] =  self.l1 * c1 
        J[1] = -self.l1 * s1 
        
        return J
        
        
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
    def F(self, q = np.zeros(1) , dq = np.zeros(1) , ddq = np.zeros(1)):
        """ Computed torques given a trajectory (inverse dynamic) """  
        
        H = self.H( q )
        C = self.C( q , dq )
        D = self.D( q , dq )
        G = self.G( q )
        
        F = np.dot( H , ddq ) + np.dot( C , dq ) + np.dot( D , dq ) + G
        
        return F
        
        
    ##############################
    def ddq(self, q = np.zeros(1) , dq = np.zeros(1) , F = np.zeros(1) , d = 0 ):
        """ Computed accelerations given torques"""  
        
        H = self.H( q )
        C = self.C( q , dq )
        D = self.D( q , dq )
        G = self.G( q )
        
        ddq = np.dot( 1./H ,  ( F - np.dot( C , dq ) - np.dot( D , dq ) - G + d ) )
        
        return ddq
        
        
    #############################
    def fc(self, x = np.zeros(2) , u = np.zeros(3) , t = 0 ):
        """ 
        Continuous time function evaluation
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        dx = np.zeros(self.n) # State derivative vector
        
        q  = x[0]
        dq = x[1]
        
        ddq = self.ddq( q , dq , u[0] , u[2] )
        
        dx[0] = dq
        dx[1] = ddq
        
        return dx
        
        
    #############################
    def show(self, q):
        """ Plot figure of configuration q """
        
        fig = plt.figure()
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
        ax.grid()
        
        pts = self.fwd_kinematic( q )
        
        line = ax.plot( pts[:,0], pts[:,1], 'o-', lw=(self.l1) )
        
        plt.show()
        
        return fig , ax, line
        
  
    
    #############################
    def plotAnimation(self, x0 = np.array([0,0]) , tf = 10 , n = 201 ,  save = False , file_name = 'RobotSim' ):
        """ Simulate and animate robot """
        
        
        # Integrate EoM
        self.S    = Simulation( self , tf , n )
        self.S.x0 = x0
        
        self.S.compute()
        
        
        
        # Compute pts localization
        self.PTS = np.zeros((2,2,n))
        
        for i in xrange(n):
            self.PTS[:,:,i] = self.fwd_kinematic( self.S.x_sol_CL[i,0] ) # Forward kinematic
            
            
        # figure
            
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
        self.ax.grid()
        
        self.line, = self.ax.plot([], [], 'o-', lw=2)
        self.time_template = 'time = %.1fs'
        self.time_text = self.ax.text(0.05, 0.9, '', transform=self.ax.transAxes)
            
        

                    
        self.ani = animation.FuncAnimation( self.fig, self.__animate__, n, interval=25, blit=True, init_func=self.__ani_init__)
        
        if save:
            self.ani.save( file_name + '.mp4' , writer = 'ffmpeg' )
            
        plt.show()
        
        return self.PTS
        
    def __ani_init__(self):
        self.line.set_data([], [])
        self.time_text.set_text('')
        return self.line, self.time_text
    
    
    def __animate__(self,i):
        thisx = self.PTS[:,0,i]
        thisy = self.PTS[:,1,i]
    
        self.line.set_data(thisx, thisy)
        self.time_text.set_text(self.time_template % (i*0.05))
        return self.line, self.time_text
        
        
    #############################
    def plotAnimation2(self, x0 = np.array([0,0]) , tf = 10 , dt = 0.05 ,  save = False , file_name = 'RobotSim' ):
        """ Simulate (EULER) and animate robot """
        
        # Integrate EoM
        self.t = np.arange( 0 , tf , dt )  
        n = self.t.size
        
        self.x_sol_CL = np.zeros((n,self.n))
        self.u_sol_CL = np.zeros((n,self.m))
        self.PTS = np.zeros((2,2,n))
        
        self.x_sol_CL[0,:] = x0
        
        for i in xrange(n):
            
            u  = self.ctl( self.x_sol_CL[i,:] , self.t[i] )
            x1 = self.fd( self.x_sol_CL[i,:] , u , dt )
            
            if i+1<n:
                self.x_sol_CL[i+1,:] = x1
            self.u_sol_CL[i,:]   = u
            self.PTS[:,:,i] = self.fwd_kinematic( self.x_sol_CL[i,0] ) # Forward kinematic
            
            
        # figure
            
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
        self.ax.grid()
        
        self.line, = self.ax.plot([], [], 'o-', lw=2)
        self.time_template = 'time = %.1fs'
        self.time_text = self.ax.text(0.05, 0.9, '', transform=self.ax.transAxes)
            
        

                    
        self.ani = animation.FuncAnimation( self.fig, self.__animate__, n, interval=25, blit=True, init_func=self.__ani_init__)
        
        if save:
            self.ani.save( file_name + '.mp4' , writer = 'ffmpeg' )
            
        plt.show()
        
        return self.PTS
        
        
    ##############################
    def plot_CL(self, CTL = None ,show = True ):
        """ 
        No arguments
        
        Create a figure with trajectories for all states and control inputs
        
        """
        
        l = self.m + self.n
        
        matplotlib.rc('xtick', labelsize=4)
        matplotlib.rc('ytick', labelsize=4)
        
        simfig , plots = plt.subplots(l, sharex=True,figsize=(4, 3),dpi=300, frameon=True)
        
        simfig.canvas.set_window_title('Closed loop trajectory')
        
        ##################################
        if not( CTL == None ):
            """ Desired trajectory """
            n = self.t.size
            
            ddq_d = np.zeros(n)
            dq_d  = np.zeros(n)
            q_d   = np.zeros(n)
            
            s      = np.zeros(n)
            ddq_r  = np.zeros(n)
            
            for i in xrange(n):
            
                ddq_d[i] , dq_d[i] , q_d[i]  = CTL.traj(  self.t[i] )
                s[i] ,  ddq_r[i]             = CTL.slidingvariables( self.x_sol_CL[i,0] , self.x_sol_CL[i,1] , self.t[i]  )
                
            plots[0].plot( self.t , q_d , 'r')
            plots[1].plot( self.t , dq_d , 'r')
            
        #####################################
        
        
        # For all states
        for i in xrange( self.n ):
            plots[i].plot( self.t , self.x_sol_CL[:,i] , 'b')
            plots[i].set_ylabel(self.state_label[i] +'\n'+ self.state_units[i] , fontsize=5)
            plots[i].grid(True)
        
        # For all outputs
        for i in xrange( self.m ):
            plots[i + self.n].plot( self.t , self.u_sol_CL[:,i] , 'b')
            plots[i + self.n].set_ylabel(self.input_label[i] + '\n' + self.input_units[i] , fontsize=5)
            plots[i + self.n].grid(True)
               
        plots[l-1].set_xlabel('Time [sec]', fontsize=5)
        

        
        
        simfig.tight_layout()
        
        if show:
            simfig.show()
        
        
        self.fig1   = simfig
        self.plots1 = plots
        
        ##############################
        # Phase plane
        
#        phasefig = plt.figure(figsize=(3, 2),dpi=300, frameon=True)
#        
#        phasefig.canvas.set_window_title('Phase Plane trajectory')
#        
#        plt.plot(self.x_sol_CL[:,0], self.x_sol_CL[:,1], 'b-') # path
#        plt.plot( self.x_sol_CL[-1,0] , self.x_sol_CL[-1,1] , 's') # end
#        
#        plt.xlabel(self.state_label[0] +'\n'+ self.state_units[0] , fontsize=5)
#        plt.ylabel(self.state_label[1] +'\n'+ self.state_units[1] , fontsize=5)
#        
#        plt.grid(True)
#        plt.tight_layout()
        
        ###################################
        # S - trajectory

        if not( CTL == None ):
            """ Desired trajectory """
        
            phasefig2 = plt.figure(figsize=(3, 2),dpi=300, frameon=True)
            
            phasefig2.canvas.set_window_title('Sliding variable')
            
            plt.plot(self.t , s, 'b-') 
            
            plt.xlabel('Time [sec]' , fontsize=5)
            plt.ylabel('Sliding variable' , fontsize=5)
            
            plt.grid(True)
            plt.tight_layout()
        
        
#########################################################################################################   
#########################################################################################################       
        
class ROneLinkManipulator( OneLinkManipulator ) :
    """ 2DOF Manipulator Class """
    
    
    ############################
    def __init__(self):
        
        OneLinkManipulator.__init__(self, 2 , 3 )
    
        self.Ia = 1
        
        self.Da = 1
        
        
        self.ubar = np.array([0,1,0])

        
    ##############################
    def Tlosses(self, dq = np.zeros(1) , ddq = np.zeros(1)):
        """ Computed torques losses given a trajectory  """  
                
        T = np.dot( self.Ia  , ddq ) + np.dot( self.Da  , dq )
        
        return T
        
        
    ##############################
    def T(self, q = np.zeros(1) , dq = np.zeros(1) , ddq = np.zeros(1) , R = 1 ):
        """ Computed acutator torques given a trajectory and gear ratio """ 
        
        F = self.F( q , dq , ddq )
        
        Tl = self.Tlosses( dq , ddq )
        
        T = np.dot( 1. / R , F ) + np.dot( R , Tl ) 
        
        return T
        
        
    ##############################
    def ddq_a(self, q = np.zeros(1) , dq = np.zeros(1) , T = np.zeros(1) , R = 1 , d = 0 ):
        """ Computed accelerations given actuator torques and gear ratio """  
        
        
        Ha = self.H( q ) + np.dot( R , np.dot( R , self.Ia ) )
        
        C  = self.C( q , dq )
        D  = self.D( q , dq )
        
        Ca =  C + D + np.dot( R , np.dot( R , self.Da ) )
        
        G  = self.G( q )
        
        ddq = np.dot( 1. / Ha  ,  ( np.dot( R , T ) - np.dot( Ca , dq ) - G  + d ) )
        
        return ddq
        
        
    #############################
    def fc(self, x = np.zeros(4) , u = np.zeros(3) , t = 0 ):
        """ 
        Continuous time function evaluation
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        dx = np.zeros(self.n) # State derivative vector
        
        q  = x[0]
        dq = x[1]
        
        ddq = self.ddq_a( q , dq , u[0] , u[1] , u[2] )
        
        dx[0] = dq
        dx[1] = ddq
        
        return dx


#########################################################################################################   
#########################################################################################################   
    

class OneManipulatorController:
    """ 2DOF Manipulator Controller """
    
    ############################
    def __init__(self, R = ROneLinkManipulator() ):
        
        
        self.R = R # Robot object
        
        self.R_default = 1
        
        self.lam = 1
        
        
        self.q0 = -3
        self.qt = 1
        self.Dt = 8
        
        self.distur = True
        
    
    ############################    
    def traj(self, t ):
        """ Trajectory """
        
        #ddq_d = 0 
        #dq_d  = 0
        #q_d   = 0
        
        #ddq_d = 0.1 
        #dq_d  = 0.1 * t
        #q_d   = 0.05 * t ** 2   - 3.14
        
        q0 = self.q0
        qt = self.qt
        Dt = self.Dt
        
        a = ( qt - q0 ) / 2.
        w = np.pi / Dt
        p = - np.pi / 2
        
        q_d    = a * ( 1 + np.sin( w * t + p ) ) + q0
        dq_d   = a * np.cos( w * t + p ) * w 
        ddq_d  = - a * np.sin( w * t + p ) * w ** 2
        
        if t>Dt:
            q_d   = qt
            dq_d  = 0
            ddq_d = 0
        
        return [ ddq_d , dq_d , q_d ] 
        
    ############################
    def slidingvariables(self, q , dq , t ):
        """ Compute sliding variables """
        
        [ddq_d,dq_d,q_d] = self.traj(t)
        
        q_e  =  q -  q_d
        dq_e = dq - dq_d
        
        s = dq_e + self.lam * q_e
        
        ddq_r = ddq_d - self.lam * dq_e
        
        return [ s , ddq_r ]
        
    ############################
    def disturbance(self, t ):
        """ disturbance """
        
        if self.distur:
        
            if t<5:
                d = np.random.normal(0, 2, 1)
                D = 10.
            elif t<6:
                d = np.random.normal(0, 20, 1)
                D = 100.
            else:
                D = 5.
                d = np.random.normal(0, 1, 1)
        
        else:
            d = 0
            D = 0            

            
        return [ d , D ]
        
        
    ############################
    def sld_controller(self, x , t ):
        """ Sliding mode controller """
        q  = x[0]
        dq = x[1]
        
        [ s , ddq_r ] = self.slidingvariables( q , dq , t )
        [ d , D ]     = self.disturbance( t )
        
        R = 5.
        
        T = self.R.T( q , dq , ddq_r , R ) - D / R * np.sign( s )

        
        u = np.array( [ T , R , d ] )
        
        return u
        
        
    ############################
    def RminC_sld_controller(self, x , t ):
        """ Sliding mode controller """
        q  = x[0]
        dq = x[1]
        
        [ s , ddq_r ] = self.slidingvariables( q , dq , t )
        [ d , D ]     = self.disturbance( t )
        
        R = np.sqrt( np.abs( ( self.R.F( q , dq , ddq_r ) - D * np.sign( s ) ) / ( self.R.Tlosses( dq , ddq_r ) + 0.1 ) )) 
        #R = np.sqrt( np.abs( ( self.R.F( q , dq , ddq_r ) ) / ( self.R.Tlosses( dq , ddq_r ) + 0.01 ) )) 
        
        if R < 0.1:
            R = 0.1
        elif R > 50:
            R = 50
        elif R < 0:
            print 'WTF'
            
        T = self.R.T( q , dq , ddq_r , R ) - D / R * np.sign( s )
        
        u = np.array( [ T , R , d ] )
        
        return u
        
    ############################
    def RminD_sld_controller(self, x , t ):
        """ Sliding mode controller """
        q  = x[0]
        dq = x[1]
        
        [ s , ddq_r ] = self.slidingvariables( q , dq , t )
        [ d , D ]     = self.disturbance( t )
        
        Rset = np.array([4,25,49])
        Q = np.zeros(3)
        
        for i in xrange(3):
            R = Rset[i]
            T = self.R.T( q , dq , ddq_r , R ) - D / R * np.sign( s )
            Q[i] = T**2
            
        R = Rset[ Q.argmin() ]
            
        T = self.R.T( q , dq , ddq_r , R ) - D / R * np.sign( s )

        u = np.array( [ T , R , d ] )
        
        return u

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    R = ROneLinkManipulator()
    
    #CTL = None
    CTL = OneManipulatorController( R )
    
    R.ctl = CTL.sld_controller
    #R.ctl = CTL.RminC_sld_controller
    #R.ctl = CTL.RminD_sld_controller
    
    R.plotAnimation2([-4,0],10)
    
    R.plot_CL( CTL )
    R.phase_plane_trajectory([0,1,0],[-4,0],10,True,True,True,True)
            