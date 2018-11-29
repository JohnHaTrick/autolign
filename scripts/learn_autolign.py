#!/usr/bin/env/ python

import numpy         as     np
import ctrl_autolign as     ctrl
from   sim_autolign  import X1, LinearSimulator

def gradDescent(guess,delta,state_1,dt,path,i):
    # minimize cost function (x_1_meas - x_1_model)^2
    #   using linear model with states [ e dPsi Uy r ]

    if not memory.state_0:		        # First iteration?
	memory.state_0 = state_1	        #   save for next time
	return guess                            #   no GD on 1st iter
    state_0 = memory.state_0
    memory.state_0 = state_1		        # set state_0 for next iter
    
    LinSim = LinearSimulator()		        # Init linear simulator
    s_0    = LinSim.getLocalState(state_0,path) # Local states to path
    s_1    = LinSim.getLocalState(state_1,path)
    Ux     = state_0['Ux']		        # speed (assumed const)

    s_1_linModel, B_matrix = LinSim.simulate_Lin(s_0,Ux,path,delta+guess,dt)

    eta   = .2/np.sqrt(i)		        # learning rate
    guess = guess + 2 * eta * dt * ( s_1 - s_1_linModel ) * B_matrix
    return guess.tolist()[0]

class memory(object):
    state_0 = dict()
