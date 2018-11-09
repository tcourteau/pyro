# -*- coding: utf-8 -*-
"""
Created on Tue Oct 23 20:45:37 2018

@author: Alexandre
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from AlexRobotics.dynamic import system


##############################################################################
        
class MechanicalSystem( system.ContinuousDynamicSystem ):
    """ 
    Mechanical system with Equation of Motion in the form of
    -------------------------------------------------------
    H(q) ddq + C(q,dq) dq + d(q,dq) + g(q) = B(q) u
    -------------------------------------------------------
    q      :  dim = (dof, 1)   : position variables 
    dq     :  dim = (dof, 1)   : velocity variables     
    ddq    :  dim = (dof, 1)   : acceleration variables
    u      :  dim = (m, 1)     : force input variables
    H(q)   :  dim = (dof, dof) : inertia matrix
    C(q)   :  dim = (dof, dof) : corriolis matrix
    B(q)   :  dim = (dof, m)   : actuator matrix
    ddq    :  dim = (dof, 1)   : acceleration variables
    d(q,dq):  dim = (dof, 1)   : state-dependent dissipative forces
    g(q)   :  dim = (dof, 1)   : state-dependent conservatives forces
    
    """
    
    ############################
    def __init__(self, dof = 1 , m = 1):
        """ """
        
        # Degree of Freedom
        self.dof = dof
        
        # Dimensions
        n = dof * 2 
        m = dof  
        p = dof * 2
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, n, m, p)
        
        # Name
        self.name = str(dof) + 'DoF Mechanical System'
        
        # Labels, bounds and units
        for i in range(dof):
            # joint angle states
            self.x_ub[i] = + np.pi * 2
            self.x_lb[i] = - np.pi * 2
            self.state_label[i] = 'Angle '+ str(i)
            self.state_units[i] = '[rad]'
            # joint velocity states
            self.x_ub[i+dof] = + np.pi * 2
            self.x_lb[i+dof] = - np.pi * 2
            self.state_label[i+dof] = 'Velocity ' + str(i)
            self.state_units[i+dof] = '[rad/sec]'
        for i in range(dof):
            self.input_label[i] = 'Torque ' + str(i)
            self.input_units[i] ='[Nm]'
        for i in range(dof):
            # joint angle states
            self.output_label[i] = 'Angle '+ str(i)
            self.output_units[i] = '[rad]'
            # joint velocity states
            self.output_label[i+dof] = 'Velocity ' + str(i)
            self.output_units[i+dof] = '[rad/sec]'
            
        # Graphic output parameters 
        self.graphic_domain  = [ (-10,10) , (-10,10) , (-10,10) ]#
            
    ###########################################################################
    # The following functions needs to be overloaded by child classes
    # to represent the dynamic of the system
    ###########################################################################
    
    ###########################################################################
    def H(self, q ):
        """ 
        Inertia matrix 
        ----------------------------------
        dim( H ) = ( dof , dof )
        
        such that --> Kinetic Energy = 0.5 * dq^T * H(q) * dq
        
        """  
        
        H = np.diag( np.ones( self.dof ) ) # Default is identity matrix
        
        return H
    
    ###########################################################################
    def C(self, q , dq ):
        """ 
        Corriolis and Centrifugal Matrix 
        ------------------------------------
        dim( C ) = ( dof , dof )
        
        such that --> d H / dt =  C + C^T
        
        
        """ 
        
        C = np.zeros( ( self.dof , self.dof ) ) # Default is zeros matrix
        
        return C
    
    ###########################################################################
    def B(self, q ):
        """ 
        Actuator Matrix  : dof x m
        """
        
        B = np.diag( np.ones( self.dof ) ) # Default is identity matrix
        
        return B
    
    ###########################################################################
    def g(self, q ):
        """ 
        Gravitationnal forces vector : dof x 1
        """
        
        g = np.zeros( self.dof ) # Default is zero vector
        
        return g
        
    ###########################################################################
    def d(self, q , dq ):
        """ 
        State-dependent dissipative forces : dof x 1
        """
        
        d = np.zeros(self.dof ) # Default is zero vector
        
        return d
        
    ###########################################################################
    # The following functions needs to be overloaded by child classes
    # to enable graphical outputs functionnalities
    ###########################################################################
    
    ###########################################################################
    def graphic_forward_kinematic(self, q ):
        """ 
        Compute points p = [x;y;z] positions given config q 
        ----------------------------------------------------
        - points of interest for ploting
        
        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines
        
        """
        
        lines_pts = [] # list of array (n_pts x 3) for each lines
        
        ###########################
        # Your graphical code here
        ###########################
            
        # simple place holder
        for i in range(self.dof):
            pts      = np.zeros(( 1 , 3 )) # array of 1 pts for the line
            pts[0,0] = q[i]                    # x cord of point 0 = q
            lines_pts.append( pts )            # list of all arrays of pts
                
        return lines_pts
    
    
    ###########################################################################
    # No need to overwrite the following functions for custom system
    ###########################################################################
    
    #############################
    def x2q( self, x ):
        """ from state vector (x) to angle and speeds (q,dq) """
        
        q  = x[ 0        : self.dof ]
        dq = x[ self.dof : self.n   ]
        
        return [ q , dq ]
        
        
    #############################
    def q2x( self, q , dq ):
        """ from angle and speeds (q,dq) to state vector (x) """
        
        x = np.zeros( self.n )
        
        x[ 0        : self.dof ] = q
        x[ self.dof : self.n   ] = dq
        
        return x
    
    ##############################
    def generalized_forces(self, q  , dq  , ddq , t = 0 ):
        """ Computed generalized forces given a trajectory (inverse dynamic) """  
        
        H = self.H( q )
        C = self.C( q , dq )
        g = self.g( q )
        d = self.g( q , dq )
                
        # Generalized forces
        forces = np.dot( H , ddq ) + np.dot( C , dq ) + g + d
        
        return forces
    
    ##############################
    def actuator_forces(self, q  , dq  , ddq , t = 0 ):
        """ Computed actuator forces given a trajectory (inverse dynamic) """  
        
        B = self.B( q )
                
        # Generalized forces
        forces = self.generalized_forces( q , dq , ddq , t )
        
        # Actuator forces
        u = np.dot( np.linalg.inv( B ) , forces )
        
        return u
    
    
    ##############################
    def ddq(self, q , dq , u , t = 0 ):
        """ Computed accelerations given actuator forces u """  
        
        H = self.H( q )
        C = self.C( q , dq )
        g = self.g( q  )
        d = self.d( q , dq)
        B = self.B( q )
        
        ddq = np.dot( np.linalg.inv( H ) ,  ( np.dot( B , u )  - np.dot( C , dq ) - g - d ) )
        
        return ddq
    
    
    ###########################################################################
    def f(self, x , u , t = 0 ):
        """ 
        Continuous time foward dynamics evaluation
        
        dx = f(x,u,t)
        
        INPUTS
        x  : state vector             n x 1
        u  : control inputs vector    m x 1
        t  : time                     1 x 1
        
        OUPUTS
        dx : state derivative vectror n x 1
        
        """
        
        [ q , dq ] = self.x2q( x )       # from state vector (x) to angle and speeds (q,dq)
        
        ddq = self.ddq( q , dq , u , t ) # compute joint acceleration 
        
        dx = self.q2x( dq , ddq )        # from angle and speeds diff (dq,ddq) to state vector diff (dx)
        
        return dx
    
    
    ###########################################################################
    def kinetic_energy(self, q  , dq ):
        """ Compute kinetic energy of manipulator """  
        
        e_k = 0.5 * np.dot( dq , np.dot( self.H( q ) , dq ) )
        
        return e_k
        
    
    ###########################################################################
    # Graphical output utilities
    ###########################################################################
    
    #############################################
    def show(self, q , x_axis = 0 , y_axis = 1 ):
        """ Plot figure of configuration q """
        
        self.showfig = plt.figure(figsize=(4, 3), dpi=300)
        self.showfig.canvas.set_window_title('2D Configuration of ' + 
                                            self.name )
                                            
        self.showax = self.showfig.add_subplot(111,
                                            autoscale_on=False, 
                                            xlim=self.graphic_domain[x_axis],
                                            ylim=self.graphic_domain[y_axis] )
        self.showax.grid()
        
        lines_pts      = self.graphic_forward_kinematic( q )
        self.showlines = []
        
        for pts in lines_pts:
            x_pts = pts[:, x_axis ]
            y_pts = pts[:, y_axis ]
            line  = self.showax.plot( x_pts, y_pts, 'o-')
            self.showlines.append( line )
        
        plt.show()
        
    
    #############################################
    def show3(self, q ):
        """ Plot figure of configuration q """
        
        self.show3fig = plt.figure(figsize=(4, 3), dpi=300)
        self.showfig.canvas.set_window_title('3D Configuration of ' + 
                                            self.name )
        self.show3ax = self.show3fig.gca(projection='3d')
        
        lines_pts      = self.graphic_forward_kinematic( q )
        self.show3lines = []
        
        for pts in lines_pts:
            x_pts = pts[:, 0 ]
            y_pts = pts[:, 1 ]
            z_pts = pts[:, 2 ]
            line  = self.show3ax.plot( x_pts, y_pts, z_pts, 'o-')
            self.show3lines.append( line )
            
        self.show3ax.set_xlim3d( self.graphic_domain[0] )
        self.show3ax.set_xlabel('X')
        self.show3ax.set_ylim3d( self.graphic_domain[1] )
        self.show3ax.set_ylabel('Y')
        self.show3ax.set_zlim3d( self.graphic_domain[2] )
        self.show3ax.set_zlabel('Z')
        
        plt.show()
        
    
    #############################
    def plot_animation(self, x0 , tf = 10 , n = 10001 , solver = 'ode',  save = False , file_name = 'RobotSim'  ):
        """ Simulate and animate system """
        
        self.compute_trajectory( x0 , tf , n , solver )
        
        self.animate_sim( 1.0 , save , file_name )
        
        
                
    ##############################
    def animate_sim(self, time_factor_video =  1.0 , save = False , file_name = 'RobotSim' ):
        """ 
        Show Animation of the simulation 
        ----------------------------------
        time_factor_video < 1 --> Slow motion video        
        
        """  
        
        # Compute pts localization
        self.ani_data = []
        
        for i in range( self.sim.n ):
            [ q , dq ]      = self.x2q(  self.sim.x_sol[i,:]  )
            lines_pts       = self.graphic_forward_kinematic( q )
            self.ani_data.append(lines_pts)
            
        # Init figure
        self.ani_fig = plt.figure(figsize=(4, 3), dpi=300 )
        self.ani_fig.canvas.set_window_title('2D Animation of ' + 
                                            self.name )
        
        self.ani_ax = self.ani_fig.add_subplot(111,
                                            autoscale_on=True, 
                                            xlim=self.graphic_domain[0],
                                            ylim=self.graphic_domain[1] )
        self.ani_ax.tick_params(axis='both', which='both', labelsize=
                                self.sim.fontsize)

        self.ani_ax.grid()
                
        # init lines
        self.lines = []
        # for each lines of the t=0 data point
        for j, line_pts in enumerate(self.ani_data[0]):
            thisx = line_pts[:,0]
            thisy = line_pts[:,1]
            line = self.ani_ax.plot(thisx, thisy, 'o-')
            self.lines.append( line )
        
        self.time_template = 'time = %.1fs'
        self.time_text = self.ani_ax.text(0.05, 0.9, '', transform=self.ani_ax.transAxes)
            
        self.ani_fig.tight_layout()
        
        
        # Animation
        inter      =  40.             # ms --> 25 frame per second
        frame_dt   =  inter / 1000. 
        
        if ( frame_dt * time_factor_video )  < self.sim.dt :
            # Simulation is slower than video
            self.skip_steps = 1                                         # don't skip steps
            inter           = self.sim.dt * 1000. / time_factor_video   # adjust frame speed to simulation
            n_frame         = self.sim.n
            
        else:
            # Simulation is faster than video
            self.skip_steps =  int( frame_dt / self.sim.dt * time_factor_video ) # --> number of simulation frame to skip between video frames
            n_frame         =  int( self.sim.n / self.skip_steps )               # --> number of video frames
        
        # ANIMATION
        # blit=True option crash on mac
        #self.ani = animation.FuncAnimation( self.ani_fig, self.__animate__, n_frame , interval = inter , init_func=self.__ani_init__ , blit=True )
        self.ani = animation.FuncAnimation( self.ani_fig, self.__animate__, n_frame , interval = inter , init_func=self.__ani_init__ )
        
        if save:
            self.ani.save( file_name + '.mp4' ) # , writer = 'mencoder' )
        
    #####################################    
    def __ani_init__(self):
        for line in self.lines:
            line[0].set_data([], [])
        self.time_text.set_text('')
        return self.lines, self.time_text
    
    ######################################
    def __animate__(self,i):

        for j, line in enumerate(self.lines):
            thisx = self.ani_data[i * self.skip_steps][j][:,0]
            thisy = self.ani_data[i * self.skip_steps][j][:,1]
            line[0].set_data(thisx, thisy)
        self.time_text.set_text(self.time_template % ( i * self.skip_steps * self.sim.dt ))
        return self.lines, self.time_text
        
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    m = MechanicalSystem( 2 )
    m.ubar = np.array([1,2])
    x0     = np.array([0,0,0,0])
    
    m.plot_trajectory( x0 )
    
    m.show( np.array([1,2]))
    #m.show3( np.array([-0.5,1.5]))
    
    m.animate_sim()
        