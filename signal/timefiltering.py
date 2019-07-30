# -*- coding: utf-8 -*-
"""
Created on Tue Aug 30 17:36:01 2016

@author: agirard
"""

import scipy.signal as signal


'''
################################################################################
'''


class LowPassFilter:
    """ Low pass filter function for discrete data array """
    
    ############################
    def __init__(self, fc = 10 , dt = 0.01 ):
        """
        Low Pass filter
        
        fc : cutoff   freq [Hz]
        fs : sampling fred [Hz]
        dt : sampling period [sec]
        
        ------------------
        use:
        
        filtered_data = low_pass.filter_array( data )
        
        """
        
        self.fc    = fc
        self.fs    = 1.0 / dt
        self.order = 2
        
        self.setup()
        
    ############################
    def setup(self):
        """ compute intermediate parametes """
        
        wn = 2 * self.fc / self.fs
        
        self.b , self.a  = signal.butter( self.order, wn, btype = 'low' )
        
        
    ############################
    def set_freq_to(self, fc = 10 , dt = 0.01 ):
        """ Recomputed filter given a new cutoff fred """
        
        self.fc    = fc
        self.fs    = 1.0 / dt
        self.setup()
        
        
    ############################
    def filter_array(self, data ):
        
        #self.filtered_data = signal.lfilter( self.b, self.a, data )
        self.filtered_data = signal.filtfilt( self.b, self.a, data )
        
        return self.filtered_data
        
        
        
