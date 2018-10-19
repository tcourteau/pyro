# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 15:34:29 2017

@author: alxgr
"""

import os
import importlib


os.chdir('students/')

for file in os.listdir():
    name, extension = os.path.splitext( file )
    code2test = importlib.import_module(name)
    
    u = code2test.ctl(0,0)
    
    print(name,u)