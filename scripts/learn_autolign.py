#!/usr/bin/env/ python

import numpy        as     np
from   sim_autolign import X1Simulator
import ctrl_autolign as ctrl

def gradDescent(guess,delta,state_1,dt,path):
    # minimize cost function (x_1_meas - x_1_model)^2
    #   using linear model with states [ e dPsi Uy r ]

    if not memory.state_0:		    # First iteration?
	memory.state_0 = state_1	    #   save for next time
	return guess

    X1 = X1Simulator()			    # Vehicle parameters:
    a  = X1.a
    b  = X1.b
    M  = X1.M
    Iz = X1.Iz
    Cf = X1.Cf
    Cr = X1.Cr
					    # State parameters
    s_0 = np.array([ctrl.calcLateralError(path,memory.state_0),
		    ctrl.calcHeadingError(path,memory.state_0),
		    memory.state_0['Uy']                      ,
		    memory.state_0['r']                       ])
    s_1 = np.array([ctrl.calcLateralError(path,state_1),
		    ctrl.calcHeadingError(path,state_1),
		    state_1['Uy']                      ,
		    state_1['r']                       ])
    memory.state_0 = state_1		    #   set state_0 for next iter
    Ux      = state_1['Ux']		    #   speed (assumed const)

    I       = np.identity(4)		    # Learning parameters
    eta     = .01				    #   learning rate
    Q       =  (Cr*b   - Cf*a  )/M /Ux
    R	    = -(Cf*a*a + Cr*b*b)/Iz/Ux
    C	    =       Cf + Cr
    A = np.matrix([
		    [0, Ux,      1,    0],
		    [0, 0 ,      0,    1],
		    [0, 0 ,-C/M/Ux, Q-Ux],
		    [0, 0 ,   Q/Iz,    R]
		  ])			    # dynamics
    B = np.matrix([
		    [      0,       0,       0,       0],
		    [      0,       0,       0,       0],
		    [   Cf/M,    Cf/M,    Cr/M,    Cr/M],
		    [a*Cf/Iz, a*Cf/Iz,-b*Cr/Iz,-b*Cr/Iz]
		  ])			    # control
    C = np.matrix([
		    0,
		    path['k']*Ux,
		    0,
		    0
		  ])			    # affine
    s_1_model = (I+dt*A).dot(s_0) + dt*(B.dot(delta+guess)+C)
    
    guess = guess + 2*eta*dt*(s_1 - s_1_model)*B
    return guess.tolist()[0]

class memory(object):
    state_0 = dict()
