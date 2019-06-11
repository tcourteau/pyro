# -*- coding: utf-8 -*-


from scipy import optimize


def f(x):   
    return .5*(1 - x[0])**2 + (x[1] - x[0]**2)**2 + x[2]**2 + x[3]**2 + x[4]**2 



res = optimize.fmin_l_bfgs_b(f, [2, 2, 3 , 4 , 5 , 6], approx_grad=1 )  