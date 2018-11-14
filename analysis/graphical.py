# -*- coding: utf-8 -*-
"""
Created on Thu Nov  8 21:05:55 2018

@author: Alexandre
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3

from AlexRobotics.dynamic import system

##############################################################################
        
class Animator:
    """ 

    """
    
    ############################
    def __init__(self, sys ):
        """
        
        sys = system.ContinuousDynamicSystem()
        
        sys needs to implement:
        
        get configuration from states, inputs and time
        ----------------------------------------------
        q             = sys.xut2q( x , u , t )
        
        get graphic output list of lines from configuration
        ----------------------------------------------
        lines_pts     = sys.forward_kinematic_lines( q )
        
        get graphic domain from configuration
        ----------------------------------------------
        ((,),(,),(,)) = sys.forward_kinematic_domain( q )
        
        """
        
        self.sys = sys
        
        self.x_axis = 0
        self.y_axis = 1
        
        # Params
        self.figsize   = (4, 3)
        self.dpi       = 300
        self.linestyle = 'o-'
        self.fontsize  = 5
        
    
    #############################################
    def show(self, q , x_axis = 0 , y_axis = 1 ):
        """ Plot figure of configuration q """
        
        # Update axis to plot in 2D
        
        self.x_axis = x_axis
        self.y_axis = y_axis
        
        # Get data
        lines_pts      = self.sys.forward_kinematic_lines( q )
        domain         = self.sys.forward_kinematic_domain( q )
        
        # Plot
        self.showfig = plt.figure(figsize=self.figsize, dpi=self.dpi)
        self.showfig.canvas.set_window_title('2D Configuration of ' + 
                                            self.sys.name )
        self.showax = self.showfig.add_subplot(111, autoscale_on=False )
        self.showax.grid()
        self.showax.axis('equal')
        self.showax.set_xlim(  domain[x_axis] )
        self.showax.set_ylim(  domain[y_axis] )
        
        self.showlines = []
        
        for pts in lines_pts:
            x_pts = pts[:, x_axis ]
            y_pts = pts[:, y_axis ]
            line  = self.showax.plot( x_pts, y_pts, self.linestyle)
            self.showlines.append( line )
        
        plt.show()
        
    
    #############################################
    def show3(self, q ):
        """ Plot figure of configuration q """
        
        # Get data
        lines_pts      = self.sys.forward_kinematic_lines( q )
        domain         = self.sys.forward_kinematic_domain( q )
        
        # Plot
        self.show3fig = plt.figure(figsize=self.figsize, dpi=self.dpi)
        self.show3fig.canvas.set_window_title('3D Configuration of ' + 
                                            self.sys.name )
        self.show3ax = self.show3fig.gca(projection='3d')
                
        self.show3lines = []
        
        for pts in lines_pts:
            x_pts = pts[:, 0 ]
            y_pts = pts[:, 1 ]
            z_pts = pts[:, 2 ]
            line  = self.show3ax.plot( x_pts, y_pts, z_pts, self.linestyle)
            self.show3lines.append( line )
            
        self.show3ax.set_xlim3d( domain[0] )
        self.show3ax.set_xlabel('X')
        self.show3ax.set_ylim3d( domain[1] )
        self.show3ax.set_ylabel('Y')
        self.show3ax.set_zlim3d( domain[2] )
        self.show3ax.set_zlabel('Z')
        
        plt.show()
        
    
    #############################
    def plot_animation(self, x0 , tf = 10 , n = 10001 , solver = 'ode',  save = False , file_name = 'Ani'  ):
        """ Simulate and animate system """
        
        self.sys.compute_trajectory( x0 , tf , n , solver )
        
        self.animate_simulation( 1.0 , save , file_name )
        
        
                
    ##############################
    def animate_simulation(self, time_factor_video =  1.0 , is_3d = False, save = False , file_name = 'Animation' ):
        """ 
        Show Animation of the simulation 
        ----------------------------------
        time_factor_video < 1 --> Slow motion video        
        
        """  
        self.is_3d = is_3d
        
        # Init list
        self.ani_lines_pts = []
        self.ani_domains   = []
        
        # For all simulation data points
        for i in range( self.sys.sim.n ):
            # Get configuration q from simulation
            q               = self.sys.xut2q(self.sys.sim.x_sol[i,:] ,
                                             self.sys.sim.u_sol[i,:] , 
                                             self.sys.sim.t[i] )
            # Compute graphical forward kinematic
            lines_pts       = self.sys.forward_kinematic_lines( q )
            domain          = self.sys.forward_kinematic_domain( q )
            # Save data in lists
            self.ani_lines_pts.append(lines_pts)
            self.ani_domains.append(domain)
            
        # Init figure
        self.ani_fig = plt.figure(figsize=self.figsize, dpi=self.dpi )
        
        
        if is_3d:
            self.ani_ax = p3.Axes3D( self.ani_fig )
            self.ani_ax.set_xlim3d(self.ani_domains[0][0])
            self.ani_ax.set_xlabel('X')
            self.ani_ax.set_ylim3d(self.ani_domains[0][1])
            self.ani_ax.set_ylabel('Y')
            self.ani_ax.set_zlim3d(self.ani_domains[0][2])
            self.ani_ax.set_zlabel('Z')
            self.ani_fig.canvas.set_window_title('3D Animation of ' + 
                                            self.sys.name )
        else:
            self.ani_ax = self.ani_fig.add_subplot(111, autoscale_on=True)
            self.ani_ax.axis('equal')
            self.ani_ax.set_xlim(  self.ani_domains[0][self.x_axis] )
            self.ani_ax.set_ylim(  self.ani_domains[0][self.y_axis] )
            self.ani_fig.canvas.set_window_title('2D Animation of ' + 
                                            self.sys.name )
            
        self.ani_ax.tick_params(axis='both', which='both', labelsize=
                                self.fontsize)
        self.ani_ax.grid()
                
        # Plot lines at t=0
        self.lines = []
        # for each lines of the t=0 data point
        for j, line_pts in enumerate(self.ani_lines_pts[0]):
            if is_3d:
                thisx = line_pts[:,0]
                thisy = line_pts[:,1]
                thisz = line_pts[:,2]
                line, = self.ani_ax.plot(thisx, thisy, thisz, self.linestyle)
                self.time_text = self.ani_ax.text(0, 0, 0, 'time =', 
                                                  transform=self.ani_ax.transAxes)
            else:
                thisx = line_pts[:,self.x_axis]
                thisy = line_pts[:,self.y_axis]
                line, = self.ani_ax.plot(thisx, thisy, self.linestyle)
                self.time_text = self.ani_ax.text(0.05, 0.9, 'time =', 
                                                  transform=self.ani_ax.transAxes)
                self.ani_fig.tight_layout()
            self.lines.append( line )
        
        self.time_template = 'time = %.1fs'
        
        # Animation
        inter      =  40.             # ms --> 25 frame per second
        frame_dt   =  inter / 1000. 
        
        if ( frame_dt * time_factor_video )  < self.sys.sim.dt :
            # Simulation is slower than video
            self.skip_steps = 1                                         # don't skip steps
            inter           = self.sys.sim.dt * 1000. / time_factor_video   # adjust frame speed to simulation
            n_frame         = self.sys.sim.n
            
        else:
            # Simulation is faster than video
            self.skip_steps =  int( frame_dt / self.sys.sim.dt * time_factor_video ) # --> number of simulation frame to skip between video frames
            n_frame         =  int( self.sys.sim.n / self.skip_steps )               # --> number of video frames
        
        # ANIMATION
        # blit=True option crash on mac
        #self.ani = animation.FuncAnimation( self.ani_fig, self.__animate__, n_frame , interval = inter , init_func=self.__ani_init__ , blit=True )
        if self.is_3d:
            self.ani = animation.FuncAnimation( self.ani_fig, self.__animate__, n_frame , interval = inter )
        else:
            self.ani = animation.FuncAnimation( self.ani_fig, self.__animate__, n_frame , interval = inter , init_func=self.__ani_init__ )
        
        if save:
            self.ani.save( file_name + '.mp4' ) # , writer = 'mencoder' )
        
    #####################################    
    def __ani_init__(self):
        for line in self.lines:
            line.set_data([], [])
        self.time_text.set_text('')
        return self.lines, self.time_text, self.ani_ax
    
    ######################################
    def __animate__(self,i):
        
        # Update lines
        for j, line in enumerate(self.lines):
            if self.is_3d:
                thisx = self.ani_lines_pts[i * self.skip_steps][j][:,0]
                thisy = self.ani_lines_pts[i * self.skip_steps][j][:,1]
                thisz = self.ani_lines_pts[i * self.skip_steps][j][:,2]
                line.set_data(thisx, thisy)
                line.set_3d_properties(thisz)
            else:
                thisx = self.ani_lines_pts[i * self.skip_steps][j][:,self.x_axis]
                thisy = self.ani_lines_pts[i * self.skip_steps][j][:,self.y_axis]
                line.set_data(thisx, thisy)
            
        # Update time
        self.time_text.set_text(self.time_template % ( i * self.skip_steps * self.sys.sim.dt ))
        
        # Update domain
        if self.is_3d:
            self.ani_ax.set_xlim3d(self.ani_domains[i * self.skip_steps][0])
            self.ani_ax.set_ylim3d(self.ani_domains[i * self.skip_steps][1])
            self.ani_ax.set_zlim3d(self.ani_domains[i * self.skip_steps][2])
        else:
            self.ani_ax.set_xlim(  self.ani_domains[i * self.skip_steps][self.x_axis] )
            self.ani_ax.set_ylim(  self.ani_domains[i * self.skip_steps][self.y_axis] )
        
        return self.lines, self.time_text, self.ani_ax
    
    
    
    
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    from AlexRobotics.dynamic import pendulum
    from AlexRobotics.dynamic import vehicle
    
    #sys  = SinglePendulum()
    #x0   = np.array([0,1])
    
    #sys.plot_trajectory( x0 )
    
    #sys.show( np.array([0]))
    #sys.show3( np.array([0]))
    
    #sys.animate_simulation()
    
    sys = pendulum.DoublePendulum()
    x0 = np.array([0.1,0.1,0,0])
    
    #sys.show(np.array([0.1,0.1]))
    #sys.show3(np.array([0.1,0.1]))
    
    is_3d = False
    
    sys.plot_trajectory( x0 , 20)
    
    a = Animator(sys)
    a.animate_simulation(1,is_3d)
    
    sys = vehicle.KinematicBicyleModel()
    sys.ubar = np.array([1,0.01])
    x0 = np.array([0,0,0])
    
    b = Animator(sys)
    sys.plot_trajectory( x0 , 100 )
    b.animate_simulation(10,is_3d)