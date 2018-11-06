# -*- coding: utf-8 -*-
"""
Created on Wed Oct 24 13:07:39 2018

@author: nvidia
"""

import numpy as np


from AlexRobotics.dynamic import mechanical

##############################################################################
        
class SinglePendulum( mechanical.MechanicalSystem ):
    """ 

    """
    
    ############################
    def __init__(self):
        """ """
               
        # initialize standard params
        mechanical.MechanicalSystem.__init__(self, 1)
        
        # Name
        self.name = 'Single Pendulum'
        
        # params
        self.setparams()
        
            
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        # kinematic
        self.l1  = 2 
        self.lc1 = 1
        
        # dynamic
        self.m1       = 1
        self.I1       = 1
        self.gravity  = 9.81
        self.d1       = 0
        
        
    ##############################
    def trig(self, q ):
        """ Compute cos and sin """
        
        c1  = np.cos( q )
        s1  = np.sin( q )

        
        return [c1,s1]
    
    ###########################################################################
    def H(self, q ):
        """ 
        Inertia matrix 
        ----------------------------------
        dim( H ) = ( dof , dof )
        
        such that --> Kinetic Energy = 0.5 * dq^T * H(q) * dq
        
        """  
        
        H = np.zeros((self.dof,self.dof))
        
        H[0,0] = self.m1 * self.lc1**2 + self.I1
        
        return H
    
    ###########################################################################
    def C(self, q , dq ):
        """ 
        Corriolis and Centrifugal Matrix 
        ------------------------------------
        dim( C ) = ( dof , dof )
        
        such that --> d H / dt =  C + C^T
        
        
        """ 
        
        C = np.zeros((self.dof,self.dof))

        
        return C
    
    ###########################################################################
    def B(self, q ):
        """ 
        Actuator Matrix  : dof x m
        """
        
        B = np.diag( np.ones( self.dof ) ) #  identity matrix
        
        return B
    
    ###########################################################################
    def g(self, q ):
        """ 
        Gravitationnal forces vector : dof x 1
        """
        
        g = np.zeros( self.dof ) 
        
        [c1,s1] = self.trig( q )
        
        g[0] = self.m1 * self.gravity * self.lc1 * s1

        return g
        
    ###########################################################################
    def d(self, q , dq ):
        """ 
        State-dependent dissipative forces : dof x 1
        """
        
        d    = np.zeros( self.dof ) 
        
        d[0] = self.d1 * q[0]
        
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
        
        # ground line
        pts      = np.zeros(( 2 , 3 ))
        pts[0,:] = np.array([-10,0,0])
        pts[1,:] = np.array([+10,0,0])
        
        lines_pts.append( pts )
        
        # pendulum
        pts      = np.zeros(( 2 , 3 ))
        pts[0,:] = np.array([0,0,0])
        
        [c1,s1] = self.trig( q )
        
        pts[1,:] = np.array([ s1 * self.l1 , - c1 * self.l1 ,0])
        
        lines_pts.append( pts )
                
        return lines_pts
        
        
        
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    sys  = SinglePendulum()
    x0   = np.array([0,1])
    
    sys.plot_trajectory( x0 )
    
    #sys.show( np.array([0]))
    #sys.show3( np.array([0]))
    
    sys.animate_sim(1)