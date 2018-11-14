# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 12:19:01 2018

@author: nvidia
"""
import numpy as np

from AlexRobotics.dynamic import pendulum

##############################################################################
        
class WillPendulum( pendulum.SinglePendulum ):
    """ 

    """
                
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        # kinematic
        self.l1  = 2 
        self.lc1 = 1
        
        # dynamic
        self.m1       = 10
        self.I1       = 10
        self.gravity  = 9.81
        self.d1       = 1
        

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    wp =  WillPendulum()
    
    x0 = np.array([0,1])
    
    wp.plot_animation( x0 )