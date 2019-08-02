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



# Load pre-computed trajectory from file
this_file_dir = Path(__file__).parent
traj_file = this_file_dir.joinpath(Path('double_pendulum_rrt.npy'))
traj = plan.load_trajectory(str(traj_file))

# Controller
ctl  = nonlinear.ComputedTorqueController( sys , traj )


# goal
ctl.rbar = np.array([0,0])

# New cl-dynamic
cl_sys = ctl + sys

# Simultation
x_start  = np.array([-3.14,0,0,0])
cl_sys.plot_phase_plane_trajectory( x_start  )
cl_sys.sim.plot('xu')
cl_sys.animate_simulation()