# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 11:37:48 2018

@author: alxgr
"""

import numpy as np

from AlexRobotics.core import control

###########################################################################################
# Simple proportionnal controller
###########################################################################################
        
class ProportionnalSingleVariableController( control.StaticController ) :
    """ 
    Mother class for memoryless controllers
    ---------------------------------------
    r  : reference signal vector  k x 1
    y  : sensor signal vector     p x 1
    u  : control inputs vector    m x 1
    t  : time                     1 x 1
    ---------------------------------------
    u = c( y , r , t )
    
    m = p
    
    """
    
    ###########################################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################################
    
    
    ############################
    def __init__(self):
        """ """
        
        # Dimensions
        self.k = 1   
        self.m = 1   
        self.p = 1
        
        # Label
        self.name = 'Proportionnal Controller'
        
        # Reference signal info
        self.ref_label = ['Ref.']
        self.ref_units = ['[]']
        self.r_ub      = np.zeros(self.k) + 10 # upper bounds
        self.r_lb      = np.zeros(self.k) - 10 # lower bounds
        
        # default constant reference
        self.rbar = np.zeros(self.k)
        
        # Gains
        self.gain_p = 1
        
    
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
        
        u = np.zeros(self.m) # State derivative vector
        
        e = r - y
        u = e * self.gain_p
        
        return u





'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    from AlexRobotics.core import analysis
    from AlexRobotics.dynamic import integrator
    from AlexRobotics.core import control
    
    # Double integrator
    di = integrator.DoubleIntegrator()
    si = integrator.SimpleIntegrator()
    
    # Controller 
    psvc = ProportionnalSingleVariableController()
    psvc.gain_p = 1
    
    # New cl-dynamic
    clsi = control.ClosedLoopSystem( si ,  psvc )
    clsi.plot_phase_plane_trajectory([10],10,0,0)
    clsi.sim.plot('xu')
    
    cldi = control.ClosedLoopSystem( di ,  psvc )
    cldi.plot_phase_plane_trajectory_CL([10,0],20,0,1)
    cldi.sim.plot('xu')