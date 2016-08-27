# -*- coding: utf-8 -*-
"""
Created on Sun Aug  7 15:28:12 2016

@author: alex
"""

from AlexRobotics.dynamic  import Hybrid_Manipulator   as HM
from AlexRobotics.dynamic  import DynamicSystem        as DS

import matplotlib.pyplot as plt
import numpy             as np

""" Params """

save_fig   = True
format_fig = 'pdf'

""" Define system """

# Define dynamic system
R    =  HM.HybridOneLinkManipulator()

# Definre domain of interest
R.x_ub = np.array([ 3* np.pi  ,   2*np.pi  ])
R.x_lb = np.array([ 0 , -2* np.pi  ])


# Set of params
r    = 1   # Gear ratio
R.d1 = 0   # Load side damping
R.Da = 0   # Actuator side damping
R.I1 = 1
R.Ia = 1

R.ubar = np.array([ 0.0, r ])

""" Ploting """

R.PP =  DS.PhasePlot( R , 0 , 1 , False , True )

# Figure no 1
name = 'output/pp1' + '.' + format_fig

R.PP.color_CL   = 'b'
R.PP.linewidth  = 0.04
R.PP.headlength = 4.5
R.PP.fontsize   = 10
R.PP.dpi        = 600
R.PP.figsize    = (3,2)
R.PP.y1n        = 16
R.PP.y2n        = 16
R.PP.compute() 
R.PP.plot()
if save_fig:
    R.PP.phasefig.savefig( name , format = format_fig , bbox_inches='tight', pad_inches=0.05 )

# Figure no 2

r = 20
R.ubar = np.array([ 0.0, r ])

name = 'output/pp10' + '.' + format_fig
R.PP.compute() 
R.PP.plot()
if save_fig:
    R.PP.phasefig.savefig( name , format = format_fig, bbox_inches='tight', pad_inches=0.05)
