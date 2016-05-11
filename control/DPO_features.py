# -*- coding: utf-8 -*-
"""
Created on Sun Apr 24 16:46:01 2016

@author: agirard
"""

from AlexRobotics.control import DPO           as DPO
from AlexRobotics.dynamic import DynamicSystem as RDDS


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
       
'''
################################################################################
'''


class TD_Greedy_1DOF_Features( DPO.QLearning1DOF ):
    """ Dynamic programming for 1 DOF, with features design for manipulator class """
    
    ############################
    def __init__(self, sys , cost = 'time' , experiment_name = 'data' ):
        
        self.DS = sys # Dynamic system class
        
        ########################
        
        # Learning params
        self.alpha = 0.0005
        self.gamma = 0.7
        self.eps   = 1.0  # greedy / exploration
        self.exp_n = 0
        
        ####################
        
        #Features
        
        self.n_features = 3
        
        self.f1 = self.delta_enery_abs
        self.f2 = self.sliding_var_abs
        
        # initial weight values
        self.W = np.ones( self.n_features )
        
        #######################
        
        self.x0         = np.array([3,0])        
        self.x_target   = np.array([0,0])
        
        #########################
        
        self.dt   = 0.1       # time discretization
        self.tf   = 25
        
        self.INF  = 10         #  default large cost
        self.max_error = [0.2,0.2]  # Value of epsilon
        
        ###################################
        
        self.cost = cost
        self.experiment_name = experiment_name
        
        # Quadratic cost
        self.rho    = 0.1        
        self.w_quad = np.array([ 0.1 , 0.01 , self.rho * 0.01 ])
        
        print('Reinforcement learning TD greedy with linear features:')
        
        # Predefined cost params
        if cost == 'time':
            
            print('Minimium time optimization')
            self.g    = self.g_time
            self.h    = self.h_quad
            self.INF  = 6
            
        elif cost == 'quadratic':
            
            print('Quadratic cost optimization')
            self.g      = self.g_quadratic
            self.h      = self.h_quad       # no final cost
            self.INF    = 10
            
        elif cost == 'energy':
            
            print('Minimium energy optimization')
            self.g    = self.g_energy
            self.h    = self.h_target
            self.INF  = 6
        
        else :
            
            print('Warning: not a standar cost function')  
            
        
        self.discretizeactions()
            
    #############################
    def discretizeactions(self):
        
        self.Nu0 = 11
        self.U   = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , self.Nu0  )
        
        
    
    ######################
    def features_vector( self, x = np.zeros(2) ):
        """ features """
        
        feat = np.array([ self.f1( x )  , self.f2(x) ,  1 ])
        # feat = np.array([ self.f1( x ) , self.f2( x ) ])
        
        return feat
        
    ######################
    def j_hat( self, x = np.zeros(2) ):
        """ features """
        
        feat  = self.features_vector( x )
        j_hat =  np.dot( self.W  , feat )
        
        return j_hat

        
    ######################
    def f1( self, x = np.zeros(2) ):
        """ scalar feature #1 """
        
        return 0
        
    ######################
    def f2( self, x = np.zeros(2) ):
        """ scalar feature #2 """
        
        return 0
        
    ######################
    def f3( self, x = np.zeros(2) ):
        """ scalar feature #2 """
        
        return 0
        
        
    ######################
    def delta_enery( self, x = np.zeros(2) ):
        """ delta energy """
        
        [ e , e_k , e_p ] = self.DS.energy_values( self.x_target )
        e_target = e
        
        [ e , e_k , e_p ] = self.DS.energy_values( x )
        e_actual = e
        
        delta_enery = e_target - e_actual
        
        return delta_enery
        
    ######################
    def delta_enery_abs( self, x = np.zeros(2) ):
        """ delta energy """

        delta_enery = self.delta_enery( x  )
        
        return np.abs( delta_enery )
        
        
    ######################
    def sign_of_speed( self, x = np.zeros(2) ):
        """ sign of speed """
        
        dq = x[1]
        
        return  np.sign( dq )
        
    ######################
    def pos_err_abs( self, x = np.zeros(2) ):
        """ position error abs """

        return  np.abs( x[0] )
        
    ######################
    def speed_err_abs( self, x = np.zeros(2) ):
        """ position error abs """

        return  np.abs( x[1] )
        
    ######################
    def sliding_var_abs( self, x = np.zeros(2) ):
        """ position error abs """

        return  np.abs( x[1] + 2 * x[0] )

        
        
        
        
    #######################
    #### Modif to functions
    ##########
        
    ##############################
    def Learn( self , x = np.array([0,0]) , u = np.array([0]) , x_next = np.array([0,0]) ):
        """  """
            
        # validity of the options
        x_ok = self.DS.isavalidstate( x_next )
        u_ok = self.DS.isavalidinput( x , u  )
        
        # New Q sample

        if x_ok and u_ok:
            J_next   = self.j_hat( x_next )
            J_sample = self.g( x , u ) + self.gamma * J_next
        else:
            J_next   = self.j_hat( x_next )
            J_sample = self.INF
        
        #print "Next: ",J_next, "Sample: ", J_sample
        
        # Value function error
        error  = J_sample  -  self.j_hat( x )
        
        #print "Error: ", error, "Feature:", self.features_vector( x )
        
        # Gradient descent
        self.W      = self.W + self.alpha * error * self.features_vector( x )
        
        #print "W: ", self.W  
        
    
    ##############################
    def exploration_ctl( self , x = np.array([0,0]) , t = 0 ):
        """ Random or Optimal CTL """
        
        u = np.zeros( self.DS.m  )
        
        if np.random.uniform(0,1) < self.eps:
            
            # Current optimal behavior
            u[0] = self.greedy_policy( x , t )
        
        else:
            
            # Random exploration
            random_index = int(np.random.uniform( 0 , self.Nu0 ))
            u[0] = self.U[ random_index ]         
            
        return u
        
    ##############################
    def greedy_policy( self , x = np.array([0,0]) , t = 0 ):
        """ Optimal (so-far ) CTL """
        
        Q_list = np.zeros( self.Nu0 )
        
        # for all possible actions
        for k in xrange( self.Nu0 ):
            
            u = self.U[k]
            
            if self.DS.m == 1:
                u = np.array( [ u ] )
            
            # Compute next state for all inputs
            x_next = self.DS.fc( x , u ) * self.dt + x
            
            # validity of the options
            x_ok = self.DS.isavalidstate( x_next )
            u_ok = self.DS.isavalidinput( x , u  )

            if x_ok and u_ok:
                J_next    = self.j_hat( x_next )
                Q_list[k] = self.g( x , u ) + self.gamma * J_next
            else:
                Q_list[k] = self.INF
        
        k_star = Q_list.argmin()
        
        u = self.U[ k_star ]
            
        if self.DS.m == 1:
            u = np.array( [ u ] )
            
        return u
        
        
        
    ##############################
    def training( self ,  n_trial = 1 , random = False , show = True ):
        """ Training experiments """
        
        x0 = self.x0
        
        dt      = 0.05
        plot    = False
        n_plot  = 20.
        n_print = 1.
        
        for i in xrange( n_trial ):
            
            self.exp_n = self.exp_n + 1

            if random:
                p = np.random.uniform( x0[0] - 1 , x0[0] + 1 )
                s = np.random.uniform( x0[1] - 0.5 , x0[1] + 0.5 )
                x = np.array([p,s])
                
            else:
                x = x0
            
            if (i/n_print-int(i/n_print)) < 0.00001 :
                print 'Experiment #',self.exp_n
                print 'Weight =',self.W
            
            if (i/n_plot-int(i/n_plot)) < 0.00001 and show :
            # Show behavior so far

                plot = True
                self.DS.ctl  = self.greedy_policy
                    
                
            else:
                plot = False
                self.DS.ctl  = self.exploration_ctl
            
            
            self.experiment( x , self.tf , dt ,  plot )
        
        # Optimal policy
        self.DS.ctl  = self.greedy_policy
            
            
    #############################
    def experiment(self, x0 = np.array([0,0]) , tf = 10 , dt = 0.05 , plot = False ):
        """ Simulate (EULER) and animate robot """
        
        n  = int( ( tf + 0.0 ) / dt + 1 )
        
        self.DS.Sim = RDDS.Simulation( self.DS , tf , n , 'euler' )
        
        self.DS.Sim.x0 = x0
        
        self.DS.Sim.compute()
        
        if plot:
            self.DS.PTS = np.zeros((2,2,n))
            
            for i in xrange(n):
                            
                self.DS.PTS[:,:,i] = self.DS.fwd_kinematic( self.DS.Sim.x_sol_CL[i,0] ) # Forward kinematic

            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
            self.ax.grid()
            
            self.DS.line, = self.ax.plot([], [], 'o-', lw=2)
            self.DS.time_template = 'time = %.1fs'
            self.DS.time_text = self.ax.text(0.05, 0.9, '', transform=self.ax.transAxes)
            self.DS.ani = animation.FuncAnimation( self.fig, self.DS.__animateStop__, n, interval=25, blit=True, init_func=self.DS.__ani_init__ , repeat=False)
        
            plt.show()
            
        #Learning
        
        for i in xrange(n-1):    
            
            x      = self.DS.Sim.x_sol_CL[i,:]
            u      = self.DS.Sim.u_sol_CL[i,:]
            x_next = self.DS.Sim.x_sol_CL[i+1,:]
        
            self.Learn( x , u , x_next )
            
            
    ################################
    def plot_J_hat(self):
        """ print graphic """
        
        xname = self.DS.state_label[0] + ' ' + self.DS.state_units[0]
        yname = self.DS.state_label[1] + ' ' + self.DS.state_units[1]
        
        self.Nx0 = 100
        self.Nx1 = 100
        
        self.X = [ None , None ]
        
        self.X[0] = np.linspace( self.DS.x_lb[0]  , self.DS.x_ub[0]  , self.Nx0  )
        self.X[1] = np.linspace( self.DS.x_lb[1]  , self.DS.x_ub[1]  , self.Nx1  )
        
        self.J    = np.zeros(  ( self.Nx0  , self.Nx1 ) )
        
        ## Compute J_hat on grid
        for i in xrange(self.Nx0):
            for j in xrange(self.Nx1):
                x = np.array( [ self.X[0][i] , self.X[1][j] ])
                self.J[i,j]  = self.j_hat( x )
        
        ###################    
        
        fs = 10
        
        self.fig1 = plt.figure(figsize=(4, 4),dpi=300, frameon=True)
        self.fig1.canvas.set_window_title('Cost-to-go')
        self.ax1  = self.fig1.add_subplot(1,1,1)
        
        plt.ylabel(yname, fontsize = fs)
        plt.xlabel(xname, fontsize = fs)
        self.im1 = plt.pcolormesh( self.X[0] , self.X[1] , self.J.T )
        plt.axis([self.DS.x_lb[0] , self.DS.x_ub[0], self.DS.x_lb[1] , self.DS.x_ub[1]])
        plt.colorbar()
        plt.grid(True)
        plt.tight_layout()
            
            

        
        

       
'''
################################################################################
'''


class TD_Greedy_hybrid_1DOF_Features( TD_Greedy_1DOF_Features ):
    """ Dynamic programming for 1 DOF, with features design for manipulator class """
    
    ############################
    def __init__(self, sys , cost = 'time' , experiment_name = 'data' ):
        
        TD_Greedy_1DOF_Features.__init__(self, sys , cost , experiment_name )
        

    #############################
    def discretizeactions(self):
        
        self.Nu0 = 3
        
        self.U = np.zeros([self.Nu0 * 2 , 2])
        
        # Continuous options
        Uc   = np.linspace( self.DS.u_lb[0]  , self.DS.u_ub[0]  , self.Nu0  )
        self.U[0:self.Nu0,0]  = Uc
        self.U[self.Nu0:,0]   = Uc
        
        # Discrete options
        self.U[0:self.Nu0,1]  = 1 # Gear #1
        self.U[self.Nu0:,1]   = 5 # Gear #2
        
        
    ##############################
    def exploration_ctl( self , x = np.array([0,0]) , t = 0 ):
        """ Random or Optimal CTL """
        
        u = np.zeros( self.DS.m  )
        
        
        if np.random.uniform(0,1) < self.eps:
            
            # Current optimal behavior
            u = self.greedy_policy( x , t )
        
        else:
            
            # Random exploration
            random_index = int(np.random.uniform( 0 , self.Nu0 * 2 ))
            u            = self.U[ random_index ]         
            
        return u
        
    ##############################
    def greedy_policy( self , x = np.array([0,0]) , t = 0 ):
        """ Optimal (so-far ) CTL """
        
        Q_list = np.zeros( self.Nu0 * 2 )
        
        # for all possible actions
        for k in xrange( self.Nu0 * 2 ):
            
            u = self.U[k]
            
            if self.DS.m == 1:
                u = np.array( [ u ] )
            
            # Compute next state for all inputs
            x_next = self.DS.fc( x , u ) * self.dt + x
            
            # validity of the options
            x_ok = self.DS.isavalidstate( x_next )
            u_ok = self.DS.isavalidinput( x , u  )

            if x_ok and u_ok:
                J_next    = self.j_hat( x_next )
                Q_list[k] = self.g( x , u ) + self.gamma * J_next
            else:
                Q_list[k] = self.INF
        
        k_star = Q_list.argmin()
        
        u = self.U[ k_star ]
            
        if self.DS.m == 1:
            u = np.array( [ u ] )
            
        return u
            
    
    
'''
################################################################################
'''


class TD_Greedy_hybrid_2DOF_Features( TD_Greedy_1DOF_Features ):
    """ Dynamic programming for 1 DOF, with features design for manipulator class """
    
    ############################
    def __init__(self, sys , cost = 'time' , experiment_name = 'data' ):
        
        self.DS = sys # Dynamic system class
        
        ########################
        
        # Learning params
        self.alpha = 0.0005
        self.gamma = 0.95
        self.eps   = 0.9  # greedy / exploration
        self.exp_n = 0
        
        ####################
        
        #Features
        self.n_features = 3
        
        self.f1 = self.delta_enery_abs
        self.f2 = self.sliding_var_abs_q0
        self.f3 = self.sliding_var_abs_q1
        
        # initial weight values
        self.W = np.ones( self.n_features )
        
        #######################
        
        self.x0         = np.array([3,0,0,0])        
        self.x_target   = np.array([0,0,0,0])
        
        #########################
        
        self.dt   = 0.1       # time discretization
        self.tf   = 25
        
        self.INF  = 10         #  default large cost
        self.max_error =  0.2  # Value of epsilon
        
        ###################################
        
        self.cost = cost
        self.experiment_name = experiment_name
        
        # Quadratic cost
        self.rho      = 0.1        
        self.w_quad_x = np.array([ 0.02 , 0.02 , 0.01 , 0.01 ])
        self.w_quad_u = np.array([ 0.01 , 0.01 ]) * self.rho
        
        print('Reinforcement learning TD greedy with linear features:')
        
        # Predefined cost params
        if cost == 'time':
            
            print('Minimium time optimization')
            self.g    = self.g_time
            self.h    = self.h_quad
            self.INF  = 6
            
        elif cost == 'quadratic':
            
            print('Quadratic cost optimization')
            self.g      = self.g_quadratic
            self.h      = self.h_quad       # no final cost
            self.INF    = 10
        
        else :
            
            print('Warning: not a standard cost function')  
            
        
        self.discretizeactions()
        
        
    #############################
    def h(self,  x ):
        """ Final cost function """
        
        return 0
        
        
    #############################
    def h_zero(self,  x ):
        """ Final cost function with zero value """
        
        return 0
        
        
    #############################
    def h_target(self,  x ):
        """ Final cost function """
        # Minimum time problem h = 0 , g = 1
        
        if ( np.linalg.norm( x ) < self.max_error ):
            # On target = OK            
            cost = 0 
        else:
            # Off target = bad
            cost = self.INF
        
        return cost
        
    
    #############################
    def g(self, x , u ):
        """ step cost function """
        
        return 1
    
    #############################
    def g_time(self, x , u ):
        """ Minimum time cost """

        # On target not doing anything (don't count time at this point)
        if ( np.linalg.norm( x ) < self.max_error ):
            cost = 0
        
        # Add time for the move
        else:
            cost = self.dt # minimum time
                
        return cost

        
    #############################
    def g_quadratic(self, x , u ):
        """ Quadratic additive cost """
        
        cx = np.dot( x.T , np.dot( np.diag( self.w_quad_x )  ,  x  ) )
        cu = np.dot( u[0:2].T , np.dot( np.diag( self.w_quad_u )  ,  u[0:2]  ) )
        
        cost = ( cx + cu ) * self.dt
                
        return cost
        
        
    #############################
    def h_quad(self, x ):
        """ Quadratic additive cost """
        
        cx = np.dot( x.T , np.dot( np.diag( self.w_quad_x )  ,  x  ) )
        
        cost = ( cx ) * self.dt
                
        return cost
        

    #############################
    def discretizeactions(self):
        
        T = 5
                
        self.U = np.array([[T,0,0],[0,0,0],[-T,0,0],[0,T,0],[0,-T,0],[T,T,0],[-T,-T,0],[-T,T,0],[T,-T,0],
                  [T,0,1],[0,0,1],[-T,0,1],[0,T,1],[0,-T,1],[T,T,1],[-T,-T,1],[-T,T,1],[T,-T,1],
                  [T,0,2],[0,0,2],[-T,0,2],[0,T,2],[0,-T,2],[T,T,2],[-T,-T,2],[-T,T,2],[T,-T,2],
                  [T,0,3],[0,0,3],[-T,0,3],[0,T,3],[0,-T,3],[T,T,3],[-T,-T,3],[-T,T,3],[T,-T,3]])
#                  
        #self.U = np.array([[T,0,0],[0,0,0],[-T,0,0],[0,T,0],[0,-T,0],[T,T,0],[-T,-T,0],[-T,T,0],[T,-T,0]])
                  
        self.Nu = len( self.U )
        
        
        
    ##############################
    def exploration_ctl( self , x = np.array([0,0,0,0]) , t = 0 ):
        """ Random or Optimal CTL """
        
        u = np.zeros( self.DS.m  )
        
        
        if np.random.uniform(0,1) < self.eps:
            
            # Current optimal behavior
            u = self.greedy_policy( x , t )
        
        else:
            
            # Random exploration
            random_index = int(np.random.uniform( 0 , self.Nu ))
            u            = self.U[ random_index ]         
            
        return u
        
        
    ##############################
    def greedy_policy( self , x = np.array([0,0,0,0]) , t = 0 ):
        """ Optimal (so-far ) CTL """
        
        Q_list = np.zeros( self.Nu )
        
        # for all possible actions
        for k in xrange( self.Nu ):
            
            u = self.U[k]
            
            if self.DS.m == 1:
                u = np.array( [ u ] )
            
            # Compute next state for all inputs
            x_next = self.DS.fc( x , u ) * self.dt + x
            
            # validity of the options
            x_ok = self.DS.isavalidstate( x_next )
            u_ok = self.DS.isavalidinput( x , u  )

            if x_ok and u_ok:
                J_next    = self.j_hat( x_next )
                Q_list[k] = self.g( x , u ) + self.gamma * J_next
            else:
                Q_list[k] = self.INF
        
        k_star = Q_list.argmin()
        
        u = self.U[ k_star ]
            
        if self.DS.m == 1:
            u = np.array( [ u ] )
            
        return u
        
        
        
    ######################
    def features_vector( self, x = np.zeros(4) ):
        """ features """
        
        feat = np.array([ self.f1( x )  , self.f2( x ) , self.f3( x ) , 1 ]) # x[0] , x[1] , x[2] , x[3] , np.sin( x[0] ) , np.cos( x[0] ) , np.sin( x[1] ) , np.sin( x[1] ),  1 ])
        
        return feat
        
        
        
    ######################
    def j_hat( self, x = np.zeros(4) ):
        """ features """
        
        feat  = self.features_vector( x )
        j_hat =  np.dot( self.W  , feat )
        
        return j_hat

        
    ######################
    def f1( self, x = np.zeros(4) ):
        """ scalar feature #1 """
        
        return 0
        
        
    ######################
    def f2( self, x = np.zeros(4) ):
        """ scalar feature #2 """
        
        return 0
        
        
    ######################
    def f3( self, x = np.zeros(4) ):
        """ scalar feature #2 """
        
        return 0
        
        
    ######################
    def delta_enery( self, x = np.zeros(4) ):
        """ delta energy """
        
        #print self.x_target , x
        
        [ e , e_k , e_p ] = self.DS.energy_values( self.x_target )
        e_target = e
        
        [ e , e_k , e_p ] = self.DS.energy_values( x )
        e_actual = e
        
        delta_enery = e_target - e_actual
        
        return delta_enery
        
        
        
    ######################
    def delta_enery_abs( self, x = np.zeros(4) ):
        """ delta energy """

        delta_enery = self.delta_enery( x  )
        
        return np.abs( delta_enery )
        
        
    ######################
    def sliding_var_abs_q0( self, x = np.zeros(4) ):
        """ position error abs """

        return  np.abs( x[2] + 2 * x[0] )
        
        
    ######################
    def sliding_var_abs_q1( self, x = np.zeros(4) ):
        """ position error abs """

        return  np.abs( x[3] + 2 * x[1] )
        
        
        
    ##############################
    def training( self ,  n_trial = 1 , random = False , show = True ):
        """ Training experiments """
        
        x0 = self.x0
        
        dt      = 0.05
        plot    = False
        n_plot  = 20.
        n_print = 1.
        
        for i in xrange( n_trial ):
            
            self.exp_n = self.exp_n + 1

            if random:
                q1  = np.random.uniform( x0[0] - 1   , x0[0] + 1   )
                q2  = np.random.uniform( x0[1] - 1   , x0[1] + 1   )
                dq1 = np.random.uniform( x0[2] - 0.5 , x0[2] + 0.5 )
                dq2 = np.random.uniform( x0[3] - 0.5 , x0[3] + 0.5 )
                x   = np.array([q1,q2,dq1,dq2])
                
            else:
                x = x0
            
            if (i/n_print-int(i/n_print)) < 0.00001 :
                print 'Experiment #',self.exp_n
                print 'Weight =',self.W
            
            if (i/n_plot-int(i/n_plot)) < 0.00001 and show :
            # Show behavior so far

                plot = True
                self.DS.ctl  = self.greedy_policy
                    
                
            else:
                plot = False
                self.DS.ctl  = self.exploration_ctl
            
            
            self.experiment( x , self.tf , dt ,  plot )
        
        # Optimal policy
        self.DS.ctl  = self.greedy_policy
        
        
            
        
        