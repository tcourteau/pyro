# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:01:07 2018

@author: Alexandre
"""

###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic  import vehicle
###############################################################################

class KinematicBicyleModelCtrld(vehicle.KinematicBicyleModel):
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
        dx[2] = u[0] * np.tan( -0.1*x[1]-1*dx[1]) * ( 1. / self.lenght) 
        
        return dx

# Vehicule dynamical system
sys = KinematicBicyleModelCtrld()
    #############################

    
# Set default wheel velocity and steering angle
sys.ubar = np.array([1,0])
sys.x_ub = np.array([+5,+5,+2*3.14])
sys.x_lb = np.array([-5,-5,-2*3.14])
# Plot open-loop behavior
sys.plot_trajectory( np.array([0,2,0.1]) , 1000 )


# Plot phase plane
sys.sim.phase_plane_trajectory(1,2)

# Animate the simulation
sys.animate_simulation()