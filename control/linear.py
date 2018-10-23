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
    Simple proportionnal compensator
    ---------------------------------------
    r  : reference signal vector  k x 1
    y  : sensor signal vector     k x 1
    u  : control inputs vector    k x 1
    t  : time                     1 x 1
    ---------------------------------------
    u = c( y , r , t ) = (r - y) * gain

    """
    
    ###########################################################################################
    # The two following functions needs to be implemented by child classes
    ###########################################################################################
    
    
    ############################
    def __init__(self, k = 1):
        """ """
        
        # Dimensions
        self.k = k   
        self.m = k   
        self.p = k
        
        control.StaticController.__init__(self, self.k, self.m, self.p)
        
        # Label
        self.name = 'Proportionnal Controller'
        
        # Gains
        self.gain = 1
        
    
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
        u = e * self.gain
        
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
    si = integrator.SimpleIntegrator()
    di = integrator.DoubleIntegrator()
    ti = integrator.TripleIntegrator()
    
    # Controller 
    psvc      = ProportionnalSingleVariableController()
    psvc.gain = 1
    
    # New cl-dynamic
    clsi = control.ClosedLoopSystem( si ,  psvc )
    clsi.plot_phase_plane_trajectory_CL([10],10,0,0)
    clsi.sim.plot('xu')
    
    cldi = control.ClosedLoopSystem( di ,  psvc )
    cldi.plot_phase_plane_trajectory_CL([10,0],10,0,1)
    cldi.sim.plot('xu')
    
    clti = control.ClosedLoopSystem( ti ,  psvc )
    clti.plot_phase_plane_trajectory_CL([10,0,0],10,1,2)
    clti.sim.plot('xu')