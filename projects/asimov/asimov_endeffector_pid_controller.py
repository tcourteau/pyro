###############################################################################
import numpy as np
import matplotlib.pyplot as plt
###############################################################################
from asimov import Asimov
from pyro.control import robotcontrollers
###############################################################################
kp = 80
ki = 0
kd = 0

asimov = Asimov()  # Asimov
x0 = np.array([-np.pi/4, -3*np.pi/4, np.pi/2, 0, 0, 0])  # Position initiale

qd = np.array([0.5, -np.pi/4, 0.5])  # Cible au joint
rd = asimov.forward_kinematic_effector(qd)

ctl = robotcontrollers.EndEffectorPID(asimov, kp, ki, kd)  # Déclaration du controlleur
ctl.rbar = rd  # Cible
ctl.kd = np.array([10, 30, 30])

closed_loop_robot = ctl + asimov  # Système boucle fermé

closed_loop_robot.plot_trajectory(x0, 5)  # Calcul de la trajectoire
closed_loop_robot.sim.plot('x')  # Affichage des états
closed_loop_robot.sim.plot('u')  # Affichage des commandes
closed_loop_robot.animate_simulation(1, True )  # Animation et enregistrement

plt.show()
