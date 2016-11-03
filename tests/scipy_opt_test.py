# -*- coding: utf-8 -*-


from scipy import optimize


def f(x):   
    return .5*(1 - x[0])**2 + (x[1] - x[0]**2)**2



res = optimize.fmin_l_bfgs_b(f, [2, 2], approx_grad=1 )