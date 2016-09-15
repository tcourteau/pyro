# -*- coding: utf-8 -*-
"""
Created on Sun Aug 14 13:27:30 2016

@author: agirard
"""

from AlexRobotics.dynamic import CustomManipulator    as CM

import numpy as np

R = CM.BoeingArm()
    
x0 = np.array([2,0,0,0,0,0])
    
R.ubar =  np.array([-0.01,0,0,1])

R.plotAnimation( x0 , 10 , 201 )