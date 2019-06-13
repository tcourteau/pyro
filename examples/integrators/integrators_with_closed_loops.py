# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 10:24:51 2018

@author: Alexandre
"""

############################################################################
from pyro.dynamic  import integrator
from pyro.control  import linear
############################################################################

# Double integrator
si = integrator.SimpleIntegrator()
di = integrator.DoubleIntegrator()
ti = integrator.TripleIntegrator()

# Controller 
ctl      = linear.ProportionnalSingleVariableController()
ctl.gain = 2

# New cl-dynamic
clsi = ctl + si
clsi.plot_phase_plane_trajectory([10],10,0,0)
clsi.sim.plot('xu')

cldi = ctl + di
cldi.plot_phase_plane_trajectory([10,0],10,0,1)
cldi.sim.plot('xu')

clti = ctl + ti

clti.plot_phase_plane_trajectory_3d([0.1,0,0],10)
clti.sim.plot('xu')