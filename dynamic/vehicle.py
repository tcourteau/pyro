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
        
class KinematicBicyleModel( system.ContinuousDynamicSystem ):
    """ 
    
    dx   = V cos ( phi )
    dy   = V sin ( phi )
    dphi = V/l tan ( beta )
    
    """
    
    ############################
    def __init__(self):
        """ """
    
        # Dimensions
        self.n = 3   
        self.m = 2   
        self.p = 3
        
        # initialize standard params
        system.ContinuousDynamicSystem.__init__(self, self.n, self.m, self.p)
        
        # Labels
        self.name = 'Kinematic Bicyle Model'
        self.state_label = ['x','y','phi']
        self.input_label = ['v', 'beta']
        self.output_label = ['x','y','phi']
        
        # Units
        self.state_units = ['[m]','[m]','[rad]']
        self.input_units = ['[m/sec]', '[rad]']
        self.output_units = ['[m]','[m]','[rad]']
        
        # param
        
        self.lenght = 1
        
        # Graphic output parameters 
        self.graphic_domain  = [ (0,10) , (-5,5) , (-10,10) ]#
        
    #############################
    def f(self, x = np.zeros(3) , u = np.zeros(2) , t = 0 ):
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
        
        dx = np.zeros(self.n) # State derivative vector

        dx[0] = u[0] * np.cos( x[2] )
        dx[1] = u[0] * np.sin( x[2] )
        dx[2] = u[0] * np.tan( u[1] ) * ( 1. / self.lenght) 
        
        return dx
    
    
    #############################
    def xu2q( self, x , u ):
        """ from state vector (x) to angle and speeds (q,dq) """
        
        dx = self.f( x , u )
        
        q   = np.append(  x , u[1] )
        dq  = np.append( dx , u[0] )
        
        return [ q , dq ]
    
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
        # Top line
        ###########################
            
        pts = np.zeros((2,3))
        
        pts[0,0] = -10000
        pts[0,1] = 1
        pts[1,0] = 10000
        pts[1,1] = 1
        
        lines_pts.append( pts )
        
        ###########################
        # bottom line
        ###########################
        
        pts = np.zeros((2,3))
        
        pts[0,0] = -10000
        pts[0,1] = -1
        pts[1,0] = 10000
        pts[1,1] = -1
        
        lines_pts.append( pts )
        
        ###########################
        # Car
        ###########################
        
        pts = np.zeros((3,3))
        
        pts[0,0] = q[0]
        pts[0,1] = q[1]
        pts[1,0] = q[0] + self.lenght * np.cos( q[2] )
        pts[1,1] = q[1] + self.lenght * np.sin( q[2] )
        pts[2,0] = q[0] + self.lenght * np.cos( q[2] ) + 0.2 * self.lenght * np.cos( q[2] + q[3] )
        pts[2,1] = q[1] + self.lenght * np.sin( q[2] ) + 0.2 * self.lenght * np.sin( q[2] + q[3] )
        
        lines_pts.append( pts )
                
        return lines_pts
    
    
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
            [ q , dq ]      = self.xu2q(  self.sim.x_sol[i,:] , self.sim.u_sol[i,:] )
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
        self.ani_ax.set_xlim(  ( -10  , 10  ) )
        return self.lines, self.time_text, self.ani_ax
    
    ######################################
    def __animate__(self,i):

        for j, line in enumerate(self.lines):
            thisx = self.ani_data[i * self.skip_steps][j][:,0]
            thisy = self.ani_data[i * self.skip_steps][j][:,1]
            line[0].set_data(thisx, thisy)
        self.time_text.set_text(self.time_template % ( i * self.skip_steps * self.sim.dt ))
        
        x = self.ani_data[i * self.skip_steps][2][0,0]
        y = self.ani_data[i * self.skip_steps][2][0,1]
        #print(x)
        self.ani_ax.set_xlim(  ( -10 + x , 10 + x ) )
        self.ani_ax.set_ylim(  ( -10 + y , 10 + y ) )
        return self.lines, self.time_text, self.ani_ax
        
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    sys = KinematicBicyleModel()
    
    sys.ubar = np.array([1,0.01])
    sys.plot_trajectory( np.array([0,0,0]) , 1000 )
    
    sys.animate_sim( 100 )
        