# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:09:12 2016

@author: alex
"""
###############################################################################
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

###############################################################################
from AlexRobotics.dynamic  import system
from AlexRobotics.analysis import simulation
from AlexRobotics.control  import controller
from AlexRobotics.signal   import timefiltering
from AlexRobotics.planning import plan


###############################################################################
class OpenLoopController( controller.StaticController ) :
    """    """
    ############################
    def __init__(self, sys):
        """ """
        
        # Sys
        self.sys = sys
        
        # Dimensions
        self.k = 1   
        self.m = sys.m
        self.p = sys.p
        
        controller.StaticController.__init__(self, self.k, self.m, self.p)
        
        # Label
        self.name = 'Value Iteration Controller'
        
        # Intit
        self.solution = None
        
    
    ############################
    def t2u(self, t ):
        """ get u from solution """
        
        if self.solution == None:
            
            u = self.sys.ubar
        
        else:
            
            # Find time index
            times = self.solution[2]
            i = (np.abs(times - t)).argmin()
            
            # Find associated control input
            u = self.solution[1][:,i]
            
            # No action pass trajectory time
            if t > self.time_to_goal:
                u    = self.sys.ubar
            
        return u

    #############################
    def c( self , y , r , t  ):
        """  U depends only on time """
        
        u = self.t2u( t )
        
        return u
    
    ############################
    def load_solution(self, name = 'RRT_Solution.npy' ):
        
        RRT.load_solution( self , name )



###############################################################################
class Node:
    """ node of the random tree """
    
    ############################
    def __init__(self, x , u , t , parent ):
        
        self.x = x  # Node coordinates in the state space
        self.u = u  # Control inputs used to get there
        self.t = t  # Time when arriving at x
        self.parent = parent # Previous node
        
    
    ############################
    def distanceTo(self, x_other ):
        """ Compute distance to otherNode """
        
        return np.linalg.norm( self.x - x_other )
        
        
###############################################################################
class RRT:
    """ Rapid Random Trees search algorithm """
    
    ############################
    def __init__(self, sys , x_start ):
        
        self.sys = sys          # Dynamic system class
        
        # Init tree
        self.x_start = x_start  # origin of the graph
        self.start_node = Node( self.x_start , None , 0  , None )
        self.nodes = []
        self.nodes.append( self.start_node )
        
        # Open Loop Controller
        self.controller     = OpenLoopController( sys )
        
        # Params
        self.dt                   = 0.1
        self.INF                  = 10000
        self.eps                  = 0.001
        self.steps                = 1
        self.goal_radius          = 0.5        
        self.alpha                = 0.9    # prob of random exploration
        self.beta                 = 0.0    # prob of random u
        self.max_nodes            = 2000  # maximum number of nodes
        self.max_distance_compute = 2000   # max  nodes to check distance
        self.max_solution_time    = 100    # won"t look for longuer solution 
        
        self.test_u_domain        = False  # run a check on u input 
                
        # Ploting
        self.dyna_plot            = True
        self.dyna_node_no_update  = 100
        self.fontsize             = 5
        self.figsize              = (3, 2)
        self.dpi                  = 300
        self.x_axis               = 0  # State to plot on x axis
        self.y_axis               = 1  # State to plot on y axis
        self.z_axis               = 2  # State to plot on z axis
        
        # Debuging mode
        self.debug = False
        
        self.discretizeactions()
        
        # Init
        self.solution              = None
        self.randomized_input      = False
        
        
    #############################
    def discretizeactions(self, n = 3 ):
        """ generate the list of possible control inputs """
        
        self.u_options = [ self.sys.u_lb ,  self.sys.ubar , self.sys.u_ub ]
        
        
    ############################
    def rand_state(self):    
        """ Sample a random state """
        
        ranges = self.sys.x_ub - self.sys.x_lb
        
        x_random = np.random.rand( self.sys.n ) * ranges + self.sys.x_lb
        
        #if not( self.sys.isavalidstate( x_random ) ):
                # Sample again (recursivity)
                #x_random = self.rand_state()
        
        return x_random
        
    ############################
    
    def rand_input(self, x = 0 ):    
        """ Sample a random input """
        
        # random selection
        n_options = len( self.u_options )
        j         = np.random.randint(0,n_options)
        u         = self.u_options[j]
        
        # if u domain check is active
        if self.test_u_domain :
            # I new sample is not a valid option
            if not( self.sys.isavalidinput( x , u ) ):
                # Sample again (recursivity)
                u = self.rand_input( x )
        
        return u
        
        
    ############################
    def nearest_neighbor(self, x_target ):    
        """ Get the nearest node to a given state x """
        
        closest_node = None
        min_distance = self.INF
        
        # Number of nodes is small
        if len(self.nodes) < self.max_distance_compute + 1 :
            # Brute force        
            for node in self.nodes:
                d = node.distanceTo( x_target )
                if d < min_distance:
                    if node.t < self.max_solution_time:
                        min_distance = d
                        closest_node = node
        
        # Else if there is too many nodes to check
        else:
            # Check only last X nodes
            for i in range(self.max_distance_compute):
                node = self.nodes[ len(self.nodes) - i - 1 ]
                d = node.distanceTo( x_target )
                if d < min_distance:
                    if node.t < self.max_solution_time:
                        min_distance = d
                        closest_node = node
            
                
        return closest_node
        
        
    ############################
    def select_control_input(self, x_target , closest_node ):    
        """ pick control input """
        
        # Select a random control input
        if self.randomized_input :
            
            u          = self.rand_input( closest_node.x )
            x_next     = self.sys.x_next( closest_node.x , 
                                          u , 
                                          closest_node.t , 
                                          self.dt ,
                                          self.steps 
                                          )
            
            t_next     = closest_node.t + self.dt * self.steps
            new_node   = Node( x_next , u , t_next  , closest_node )
            
            if not( self.sys.isavalidstate( x_next ) ):
                new_node = None
        
        # Pick control input that bring the sys close to random point
        else:
            
            new_node     = None
            min_distance = self.INF
            
            for u in self.u_options:
                
                # if u domain check is active
                if self.test_u_domain:
                    # if input is not valid
                    if not( self.sys.isavalidinput( closest_node.x , u ) ):
                        # Skip this u
                        continue
                
                x_next     = self.sys.x_next( closest_node.x , 
                                              u , 
                                              closest_node.t , 
                                              self.dt ,
                                              self.steps 
                                              )
                
                t_next     = closest_node.t + self.dt * self.steps
                node       = Node( x_next , u , t_next  , closest_node )
                
                d = node.distanceTo( x_target )
                
                if ( d < min_distance ) and self.sys.isavalidstate( x_next ) :
                    min_distance = d
                    new_node     = node
                    
                
        return new_node
        
    
    ############################
    def one_step(self):    
        """ """
        x_random  = self.rand_state()
        
        node_near = self.nearest_neighbor( x_random )
        
        # if a valid neighbor was found
        if not node_near == None:
            new_node  = self.select_control_input( x_random , node_near )
            
            # if there is a valid control input
            if not new_node == None:
                self.nodes.append( new_node )
        
        
    ############################
    def compute_steps(self , n , plot = False ):    
        """ """
        for i in range( n ):
            self.one_step()
            
        if plot:
            self.plot_tree()
    
           
        
    ############################
    def find_path_to_goal(self, x_goal ):
        """ """
        
        self.x_goal  = x_goal
        
        succes   = False
        
        no_nodes = 0
        
         # Plot
        if self.dyna_plot:
            self.dyna_plot_init()
        
        while not succes:
            
            # Exploration:
            if np.random.rand() > self.alpha :
                # Try to converge to goal
                x_random = x_goal
                self.randomized_input = False
            else:
                # Random exploration
                x_random  = self.rand_state()
                
                # self.beta = probability of random exploration
                self.randomized_input = ( np.random.rand() < self.beta )

            node_near = self.nearest_neighbor( x_random )
            
            # if a valid neighbor was found
            if not node_near == None:
                new_node  = self.select_control_input( x_random , 
                                                       node_near )
                
                # if there is a valid control input
                if not new_node == None:
                    self.nodes.append( new_node )
            
                    # Distance to goal
                    d = new_node.distanceTo( x_goal )
                    
                    no_nodes = no_nodes + 1
                    
                    ##################################################
                    # Debug
                    if self.debug:
                        print(x_random, node_near.x , new_node.x )
                        wait = input("PRESS ENTER TO CONTINUE.")
                    ###################################################
                    
                    # Plot
                    if self.dyna_plot:
                        self.dyna_plot_add_node( new_node , no_nodes )
                    
                    # Succes?
                    if d < self.goal_radius:
                        succes = True
                        self.goal_node = new_node
                else:
                    pass
                    #print('on obstacle')
                    
                
            # Tree reset
            if no_nodes == self.max_nodes:
                
                print('\n-----------------------------------------------',
                      '\nRRT reseting tree',
                      '\n-----------------------------------------------')
                no_nodes = 0
                self.nodes = []
                self.nodes.append( self.start_node )
                
                if self.dyna_plot :
                    self.dyna_plot_clear()
        
        print('\n-----------------------------------------------',
              '\nRRT found a path to the goal',
              '\n-----------------------------------------------')
        
        # Compute Path
        self.compute_path_to_goal()
        
        # Give solution to controller
        self.controller.solution = self.solution
        
        # Plot
        if self.dyna_plot:
            self.dyna_plot_solution()
        
                
    ############################
    def compute_path_to_goal(self):
        """ """
        
        node = self.goal_node
        
        t      = 0
        
        x_list  = []
        u_list  = []
        t_list  = []
        dx_list = []
        
        self.path_node_list = []
        
        # Until node = start_node
        while node.distanceTo( self.x_start ) > self.eps:
            
            self.path_node_list.append( node )
            
            x_list.append( node.parent.x   )
            u_list.append( node.u     )
            t_list.append( node.parent.t   )
            
            dx_list.append( self.sys.f( node.parent.x , 
                                        node.u , 
                                        node.parent.t )  ) # state derivative
            
            # Previous Node
            node  = node.parent 

        # Arrange Time array
        t = np.array( t_list )
        t = np.flipud( t )
        
        # Arrange Input array
        u = np.array( u_list ).T
        u = np.fliplr( u )
        
        # Arrange State array
        x = np.array( x_list ).T
        x = np.fliplr( x )
        
        # Arrange State Derivative array
        dx = np.array( dx_list ).T
        dx = np.fliplr( dx )
            
        # Save solution
        self.time_to_goal    = t.max()
        self.solution        = [ x , u , t , dx ]
        self.solution_length = len( self.path_node_list )
        
        self.trajectory = plan.Trajectory( x.T , u.T , t.T , dx.T)
        
    
    ############################
    def filter_solution( self , fc = 3 ):
        
        self.low_pass_filter = timefiltering.LowPassFilter( fc , self.dt )
        
        [ x , u , t , dx ]  = self.solution
        
        x_new  = x.copy()
        u_new  = u.copy()
        dx_new = dx.copy()
        
        u_new  = self.low_pass_filter.filter_array( u )
        x_new  = self.low_pass_filter.filter_array( x  )
        dx_new = self.low_pass_filter.filter_array( dx )

        self.solution   = [ x_new , u_new , t , dx_new ]
        
        self.plot_open_loop_solution()

    
    
    ############################
    def save_solution(self, name = 'RRT_Solution.npy' ):
        
        arr = np.array( self.solution )
        
        np.save( name , arr )
        
        
    ############################
    def load_solution(self, name = 'RRT_Solution.npy' ):
        
        arr = np.load( name )
        
        self.solution        = arr.tolist()
        self.time_to_goal    = self.solution[2].max()
        self.solution_length = self.solution[2].size
        
        [ x , u , t , dx ]  = self.solution
        
        self.trajectory = plan.Trajectory( x.T , u.T , t.T , dx.T)
        

    ############################
    def plot_open_loop_solution(self):
        """ """
        
        self.sim = simulation.Simulation( self.sys , 
                                          self.time_to_goal , 
                                          self.solution_length )
        
        self.sim.t     = self.solution[2].T
        self.sim.x_sol = self.solution[0].T
        self.sim.u_sol = self.solution[1].T
        
        self.sim.plot('x') 
        self.sim.plot('u')
        
        self.sys.sim = self.sim
        
    
    ##################################################################
    ### Ploting functions
    ##################################################################            
                
    ############################
    def plot_tree(self):
        """ """
        
        # Create figure
        self.fig_tree = plt.figure(figsize=(3, 2),dpi=300, frameon=True)
        
        # Set window title
        self.fig_tree.canvas.set_window_title('RRT tree search for ' + 
                                            self.sys.name )
        
        # Create axe
        ax       = self.fig_tree.add_subplot(111)
        
        # Plot Tree
        for node in self.nodes:
            if not(node.parent==None):
                ax.plot( 
                [node.x[ self.x_axis ],node.parent.x[ self.x_axis ]] , 
                [node.x[ self.y_axis ],node.parent.x[ self.y_axis ]] , 'o-')
        
        # Plot Solution Path
        if not self.solution == None:
            for node in self.path_node_list:
                if not( node.parent == None ):
                    ax.plot( 
                    [node.x[ self.x_axis ],node.parent.x[ self.x_axis ]] , 
                    [node.x[ self.y_axis ],node.parent.x[ self.y_axis ]] , 'r')
        
        # Set axis labels
        ax.set_xlabel(
                self.sys.state_label[ self.x_axis ] + ' ' +
                self.sys.state_units[ self.x_axis ] , fontsize=self.fontsize)
        ax.set_ylabel(
                self.sys.state_label[ self.y_axis ] + ' ' + 
                self.sys.state_units[ self.y_axis ] , fontsize=self.fontsize)
        
        # Set domain
        ax.set_xlim(
                [ self.sys.x_lb[ self.x_axis ] , self.sys.x_ub[ self.x_axis ] ]
                )
        ax.set_ylim(
                [ self.sys.x_lb[ self.y_axis ] , self.sys.x_ub[ self.y_axis ] ]
                )
        
        #Set grid
        ax.grid(True)
        ax.tick_params( labelsize = self.fontsize )
        
        self.fig_tree.tight_layout()
        self.fig_tree.show()
        
        return ax
        
        
    ############################
    def plot_tree_3d(self):
        """ """
        
        # Create figure
        self.fig_tree_3d = plt.figure( figsize = self.figsize, dpi = self.dpi )
        
        # Set window title
        self.fig_tree_3d.canvas.set_window_title('RRT tree search for ' + 
                                            self.sys.name )
        
        # Create Axe
        ax = self.fig_tree_3d.gca( projection='3d' )
        
        # Plot Tree
        for node in self.nodes:
            if not( node.parent == None ):
                ax.plot( 
                [ node.x[ self.x_axis ] , node.parent.x[ self.x_axis ]] ,
                [ node.x[ self.y_axis ] , node.parent.x[ self.y_axis ]] ,
                [ node.x[ self.z_axis ] , node.parent.x[ self.z_axis ]] , 'o-')
        
        # Plot Solution Path
        if not self.solution == None:
            for node in self.path_node_list:
                if not( node.parent == None ):
                    ax.plot( 
                    [ node.x[ self.x_axis ] , node.parent.x[ self.x_axis ]] ,
                    [ node.x[ self.y_axis ] , node.parent.x[ self.y_axis ]] ,
                    [ node.x[ self.z_axis ] , node.parent.x[ self.z_axis ]] , 
                    'r')
        
        # Set domain
        ax.set_xlim3d( [ self.sys.x_lb[ self.x_axis ] ,
                         self.sys.x_ub[ self.x_axis ] ] )
        ax.set_ylim3d( [ self.sys.x_lb[ self.y_axis ] , 
                         self.sys.x_ub[ self.y_axis ] ]  )
        ax.set_zlim3d( [ self.sys.x_lb[ self.z_axis ] , 
                         self.sys.x_ub[ self.z_axis ] ]  )
    
        # Set labels
        ax.set_xlabel(
        self.sys.state_label[ self.x_axis ] + ' ' + 
        self.sys.state_units[ self.x_axis ] ,  fontsize=self.fontsize )
        
        ax.set_ylabel(
        self.sys.state_label[ self.y_axis ] + ' ' +
        self.sys.state_units[ self.y_axis ] , fontsize=self.fontsize)
        
        ax.set_zlabel(
        self.sys.state_label[ self.z_axis ] + ' ' + 
        self.sys.state_units[ self.z_axis ] , fontsize=self.fontsize)
        
        # Grid
        ax.grid(True)
        ax.tick_params( labelsize = self.fontsize )
        
        self.fig_tree_3d.tight_layout()
        self.fig_tree_3d.show()
        
        return ax
        
    
    ############################
    def dyna_plot_init(self):
        
        # Create figure
        self.fig_tree_dyna = plt.figure(figsize=(3, 2),dpi=300, frameon=True)
        
        # Set window title
        self.fig_tree_dyna.canvas.set_window_title('RRT tree search for ' + 
                                            self.sys.name )
        
        # Create axe
        ax  = self.fig_tree_dyna.add_subplot(111)
        
        self.time_template = 'Number of nodes = %i'
        self.time_text = ax.text(
                0.05, 0.05, '', transform=ax.transAxes, 
                fontsize=self.fontsize )
        
        # Set domain
        ax.set_xlim( [ self.sys.x_lb[ self.x_axis ] ,
                       self.sys.x_ub[ self.x_axis ] ] )
        ax.set_ylim( [ self.sys.x_lb[ self.y_axis ] , 
                       self.sys.x_ub[ self.y_axis ] ]  )
    
        # Set labels
        ax.set_xlabel(
        self.sys.state_label[ self.x_axis ] + ' ' + 
        self.sys.state_units[ self.x_axis ] ,  fontsize=self.fontsize )
        
        ax.set_ylabel(
        self.sys.state_label[ self.y_axis ] + ' ' +
        self.sys.state_units[ self.y_axis ] , fontsize=self.fontsize)
        
        # Grid
        ax.grid(True)
        ax.tick_params( labelsize = self.fontsize )
        
        # Compact
        self.fig_tree_dyna.tight_layout()
        
        # Save ax
        self.ax_tree_dyna   = ax
        self.node_wait_list = 0
        
        plt.ion()
        
        
     ############################
    def dyna_plot_add_node(self, node , no_nodes ):
        
        if not(node.parent==None):
                self.ax_tree_dyna.plot( 
                        [node.x[ self.x_axis ],node.parent.x[ self.x_axis ]] ,
                        [node.x[ self.y_axis ],node.parent.x[ self.y_axis ]] ,
                        'o-')
                self.time_text.set_text(self.time_template % ( no_nodes ))
                self.node_wait_list = self.node_wait_list + 1
                
                # Update graph
                if self.node_wait_list == self.dyna_node_no_update:
                    plt.pause( 0.001 )
                    self.node_wait_list = 0
                    print(
                    '\nRRT searching ( total node number = ', no_nodes, ' )'
                    )
                    
                    
    ############################
    def dyna_plot_clear(self ):
        
        self.ax_tree_dyna.clear()
        plt.close( self.fig_tree_dyna )
        self.dyna_plot_init()
                    
                    
    ############################
    def dyna_plot_solution(self ):
        
        if not self.solution == None:
            for node in self.path_node_list:
                if not(node.parent==None):
                    self.ax_tree_dyna.plot( 
                        [node.x[ self.x_axis ],node.parent.x[ self.x_axis ]] ,
                        [node.x[ self.y_axis ],node.parent.x[ self.y_axis ]] ,
                        'r')
                    
            #plt.ioff()
            self.fig_tree_dyna.show()
    
        


'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    from AlexRobotics.dynamic import pendulum
    from AlexRobotics.dynamic import vehicle
    

    sys  = pendulum.SinglePendulum()
    
    x_start = np.array([0.1,0])
    x_goal  = np.array([-3.14,0])
    
    planner = RRT( sys , x_start )
    
    planner.u_options = [
            np.array([-5]),
            np.array([-3]),
            np.array([-1]),
            np.array([ 0]),
            np.array([ 1]),
            np.array([ 3]),
            np.array([ 5])
            ]
    
    planner.alpha = 0.99
    planner.find_path_to_goal( x_goal )
    
    planner.plot_tree()
    planner.plot_open_loop_solution()
    sys.animate_simulation()
    
    planner.z_axis = 0
    planner.plot_tree_3d()