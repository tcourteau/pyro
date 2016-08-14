# -*- coding: utf-8 -*-
"""
Created on Sun Aug 14 13:27:30 2016

@author: agirard
"""

from BoeingArm import BoeingArm as BA

R = BA()
    
x0 = np.array([0.08,0,0,0,0,0])

R.plot3DAnimation( x0 , 10 )