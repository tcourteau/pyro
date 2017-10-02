# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 15:21:37 2017

@author: alxgr
"""
import numpy as np
from AlexRobotics.dynamic  import Manipulator
from AlexRobotics.control  import ComputedTorque as CTC

def ctl( x , t ):
    
    u = np.array([0,0])
    
    ################################
    # Your controller here

    R  =  Manipulator.TwoLinkManipulator()
    
    # Define controller
    ctc     = CTC.ComputedTorqueController( R )
    ctc.w0  = 1
    
    u = ctc.ctl( x , t )
    
    
    #################################
    return u