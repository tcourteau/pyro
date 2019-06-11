# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 15:21:37 2017

@author: alxgr
"""
import numpy as np
from AlexRobotics.control import linear

def ctl( x , t ):
    
    u = np.array([0,0])
    
    ################################
    # Your controller here
    
        
    kp = np.array([30,15])
    kd = np.array([5,5])
    PD = linear.PD_nDOF( kp , kd )
    
    u = PD.ctl( x , t)
    
    
    #################################
    return u