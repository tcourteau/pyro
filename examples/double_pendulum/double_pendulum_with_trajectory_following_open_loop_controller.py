# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:05:08 2018

@author: Alexandre
"""
###############################################################################
from pathlib import Path
import numpy as np
###############################################################################
from pyro.dynamic  import pendulum
from pyro.control  import nonlinear
from pyro.planning import plan
###############################################################################

sys  = pendulum.DoublePendulum()

this_file_dir = Path(__file__).parent
traj_file = this_file_dir.joinpath(Path('double_pendulum_rrt.npy'))
ctl  = plan.load_open_loop_controller(str(traj_file))

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x_start  = np.array([-3.14,0,0,0])
cl_sys.plot_phase_plane_trajectory( x_start  )
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()