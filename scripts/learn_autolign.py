#!/usr/bin/env/ python

import numpy as np

def gradDescent(guess,delta,state_1,dt):
    # minimize cost function (x_1_meas - x_1_model)^2
    #   using linear model with states [ e dPsi Ur r ]
    try:				    # Parameters
	state_0 = memory.state_0
    except:
	memory()			    #   intialize state_0 memory
	state_0 = state_1
    I   = np.identity(4)
    eta = 1				    #   learning rate
    Ux  = state['Ux']
    Q   =   (Cr*b   - Cf*a  )/m /Ux
    R   =  -(Cf*a^2 + Cr*b^2)/Iz/Ux
    C   = 2*(Cf + Cr)
    A   = np.matrix([[0, Ux,      1,    0],
		     [0, 0 ,      0,    1],
                     [0, 0 ,-C/m/Ux, Q-Ux],
                     [0, 0 ,   Q/Iz,    R]])# dynamics
    B   = np.matrix([[      0,     0,        0,       0],
		     [      0,     0,        0,       0],
                     [   Cf/m,   Cf/m,    Cr/m,    Cr/m],
                     [a*Cf/Iz,a*Cf/Iz,-b*Cr/Iz,-b*Cr/Iz]])# control
    C   = np.matrix([[0],[path['k']*Ux],[0],[0]])# affine

    guess = guess + 2*eta*dt*(state_1 - (I+dt*A)*state_0 - dt*(B*(delta+guess)+C))*B

    memory.state_0 = state_1		    # set state_0 for next iteration
    return guess

class memory(object):
    state_0 = np.array([])
