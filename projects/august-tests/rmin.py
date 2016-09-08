# -*- coding: utf-8 -*-
"""
Created on Mon Aug 08 16:35:42 2016

@author: agirard
"""

import numpy as np
import matplotlib.pyplot as plt



a = 1
b = 2

R = np.arange(-5,-0.1,0.1)
J =  a **2 / R **2 + 2 * a * b +  R ** 2 * b **2
plt.plot(R, J, 'b-') # path

R = np.arange(0.1,5,0.1)
J =  a **2 / R **2 + 2 * a * b +  R ** 2 * b **2
plt.plot(R, J, 'b-') # path