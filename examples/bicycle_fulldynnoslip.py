# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:01:07 2018

@author: Alexandre
"""

###############################################################################
import numpy as np
###############################################################################
from pyro.dynamic  import vehicle
from pyro.control  import linear
import matplotlib.pyplot as plt
###############################################################################

ctl = linear.inputControllerFI()

# Vehicule dynamical system
sys = vehicle.FullDynamicBicycleModelForceInput()

# Set default steering angle and longitudinal velocity v_x
#sys.ubar = np.array([0.2,0,1000])

#Closed-loop sys
cl_sys = ctl + sys
# Plot open-loop behavior
cl_sys.plot_trajectory( np.array([0,0,0,0,0,0]) , 100 )

# Compute tire forces and slip angles for graphical purpose
x = cl_sys.sim.x_sol
u = cl_sys.sim.u_sol 
t = cl_sys.sim.t    
 
F_nf = sys.mass*sys.g*sys.b/(sys.b+sys.a)
F_nr = sys.mass*sys.g*sys.a/(sys.b+sys.a)
max_F_f = F_nf*0.9
max_F_r = F_nr*0.9
slip_ratio_f = max_F_f/(0.12)
slip_ratio_r = max_F_r/(0.12) 
F_yf = np.zeros(len(t))
F_yr = np.zeros(len(t))
slip_f = np.zeros(len(t))
slip_r = np.zeros(len(t))
maxF_f = np.zeros(len(t))
maxF_r = np.zeros(len(t))

for i in range(len(t)-1):
    if (x[i,0] == 0):
        slip_f[i] = 0
        slip_r[i] = 0
        F_yf[i]   = 0
        F_yr[i]   = 0
    else:
        slip_f[i] = np.arctan((x[i,1]+sys.a*x[i,2])/x[i,0])-u[i,0]
        slip_r[i] = np.arctan((x[i,1]-sys.b*x[i,2])/x[i,0])
        if (slip_f[i]<-0.12):
            F_yf[i] = max_F_f
        elif (slip_f[i] > 0.12):
            F_yf[i] = -max_F_f
        else:
            F_yf[i] = -slip_ratio_f*slip_f[i]
            
        if (slip_r[i]<-0.12):
            F_yr[i] = max_F_r
        elif (slip_r[i] > 0.12):
            F_yr[i] = -max_F_r
        else:
            F_yr[i] = -slip_ratio_r*slip_r[i]
    maxF_f[i] = max_F_f
    maxF_r[i] = max_F_r

# Plot forces
figsize   = (7, 4)
dpi       = 100
plt.figure(2, figsize=figsize, dpi=dpi)
plt.title('Lateral forces for the lateral dynamic\n model with steering angle and longitudinal forces as inputs', fontsize=20)
plt.plot(t[:-1], F_yf[:-1], label = 'F_yf')
plt.plot(t[:-1], F_yr[:-1], label = 'F_yr')
plt.plot(t[:-1], maxF_f[:-1],'--', label = 'Max front force')
plt.plot(t[:-1], maxF_r[:-1],'--', label = 'Max rear force')
plt.legend(fontsize = '15')
plt.xlabel('Temps (s)')
plt.ylabel('Force (N)')

plt.show()


# Plot trajectory of the vehicle's CG
slip = 1
t_start_slip = 'No slip' #No slip
plt.figure(3,figsize=figsize, dpi=dpi)
for i in range(len(t)-1):
    if (F_yf[i]==max_F_f or F_yr[i]==max_F_r):
        if (slip == 1):            
            t_start_slip = i
            slip = 0
        else:
            t_stop_slip = i
    else:
        pass
if (t_start_slip == 'No slip'):
    plt.plot(x[:,4],x[:,5],'-b', label = 'No slip')     
else:
    plt.plot((x[t_start_slip:t_stop_slip,4]),(x[t_start_slip:t_stop_slip,5]),'-r',label='Slip')
    plt.plot((x[0:t_start_slip,4]),(x[0:t_start_slip,5]),'-b', label='No slip')
    plt.plot((x[t_stop_slip:len(t)-1,4]),(x[t_stop_slip:len(t)-1,5]),'-b')    
plt.legend(fontsize ='15')


plt.show()

# Animate the simulation
cl_sys.animate_simulation()

# Plot inputs only
cl_sys.sim.plot( plot='u' )

# Plot states only
cl_sys.sim.plot( plot='x' )