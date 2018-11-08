# -*- coding: utf-8 -*-
"""
Created on Fri Aug 12 11:14:26 2016

@author: agirard
"""

import numpy as np
from scipy.interpolate import interp1d

l2 = 0.3
h1 = 0.1
l3 = 0.2

def l1_from3( theta3 = 0 ):
    """ Inverse kin of first link """
    
    c3 = np.cos( theta3 )
    s3 = np.sin( theta3 )
    
    l1 = l3 * s3 + np.sqrt( l2 **2 - ( l3 * c3 - h1 ) **2    )
    
    return l1
    

x = np.arange( -np.pi * 0.5 , np.pi * 0.25 , 0.01)
y = x.copy()

for i in range( x.size ):
    y[i] = l1_from3( x[i] )
    
plt.plot(y, x, 'b-')

q3from_l1 = interp1d(y,x)

L1 = np.arange( 0.10, 0.40 , 0.01)
Q3 = L1.copy()

for i in range( L1.size ):
    Q3[i] = q3from_l1( L1[i] )
    
plt.plot(L1, Q3, 'r-')