# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

import numpy as np
import os
import importlib


from AlexRobotics.dynamic  import Manipulator 

###########################
# Load 2 dof robot
###########################

x_start = [-1,0.3,0,0]

###########################
# test student controller
###########################


os.chdir('students/')

for file in os.listdir():
    
    Robot  =  Manipulator.TwoLinkManipulator()
    
    name, extension = os.path.splitext( file )
    code2test = importlib.import_module(name)
    
    # Asign sudent controller to simulator
    Robot.ctl = code2test.ctl
    
    # Simulation
    Robot.plotAnimation( x_start , tf=5, n=10001, solver='euler')
    
    Robot.fig.canvas.set_window_title(name) 

    print(name,' Integral Cost:', Robot.Sim.J, ' Note:', max([0,100-Robot.Sim.J*0.05]))
    
    