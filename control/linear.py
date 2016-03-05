# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 14:59:30 2016

@author: alex
"""

import numpy as np

'''
################################################################################
'''

    
class PD:
    """ Feedback law  """
    ############################
    def __init__( self , kp , kd ):
        """ """
        self.kp = kp
        self.kd = kd
        
    ############################
    def u( self , x , t = 0 ):
        """ 
        u = f( x , t ) 
        x = [ position ; speed ]
        
        """
        u = np.zeros(1)
        
        u[0] =  -1 * ( self.kp * x[0] + self.kd * x[1] )
        
        return u