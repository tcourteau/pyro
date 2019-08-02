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


# Controller
this_file_dir = Path(__file__).parent
traj_file = this_file_dir.joinpath(Path('double_pendulum_rrt.npy'))
traj = plan.load_trajectory(str(traj_file))
ctl  = nonlinear.SlidingModeController( sys , traj )

# goal
ctl.rbar = np.array([0,0])

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x_start  = np.array([-3.14,0,0,0])
cl_sys.plot_trajectory( x_start , 10 , 10001, 'euler')
cl_sys.sim.phase_plane_trajectory(0,2)
cl_sys.animate_simulation()