#!/usr/bin/env python

import sys
import time
import math
import numpy          as np
import ctrl_autolign  as ctrl
#import sim_autolign   as sim
from sim_autolign import X1Simulator
import learn_autolign as learn
import matplotlib.pyplot as plt

try: 
    import ros_autolign
except:
    print "\nROS not available. Simulation only."

# High-level script for the automatic alignment of X1's wheels
#   README to get started

def simLoop():
    state = sim.getState()
    del_cmd  = ctrl.lookAheadCtrl(path,state)	# calc steer cmds
    del_cmd += guess				# apply misalignment guess
    Fxr      = ctrl.PI_Ctrl(path,state)		# calc longitudinal cmd (rear axle)
    print "Command %.2f rad steering and %.2f N throttle" % (float(del_cmd[0]),Fxr)
    state = sim.simulateX1(del_cmd[0:4], 1000*Fxr)			# simulate
    print learn.gradientDescent() + '\n'	# calc RL
    #print "E:",state['E']," N:",state['N']," Psi:",state['psi']," Ux:",state['Ux']," Uy:",state['Uy']," r:",state['r']
    time.sleep(.01)
    return state

def expLoop():
    # calc cmds
    # apply misalignment guess
    # send to X1 to execute
    # receive X1 state update
    # calc RL
    return -1

if __name__ == '__main__':			# main function
    print "\nStarting %s" % sys.argv[0][-11:-3]	# Welcome
    try:                                        # Did user enter a terminal argument?
        mode = sys.argv[1]			# choose simulate or experiment mode
        if mode == 'sim' or mode == 'exp':
            print "in %s mode\n" % sys.argv[1]
        else:
            print "in sim mode (by default)\n"
            mode = 'sim'
    except:
        print "in sim mode (by default)\n"
        mode = 'sim'

    # some global parameters and initializations
    dt    = .1					# 100 msec per trial
    T     =  1					# 10 sec experiment / simulation
    N     = int(T/dt)				# number of trials
    guess = [[np.nan for x in range(4)] \
	             for y in range(N)]		# initialize misalignment guesses
    guess[0][:] = [0,0,0,0]			#   to zero [FL, FR, RL, RR]
    path  = ctrl.loadPath_VAIL()		# get path

    if mode == 'exp':
        ros_autolign.listener()			# collect initial state
        while(1):
            expLoop()				# run experiment loop

    elif mode == 'sim':
        misalign = np.random.uniform(-1,1,4) \
                 * math.pi/180			# [rad] init true misalignment
	print "X1's simulated initial misalignment is: " + str(misalign) + ' radians\n'
        state = {'E'  : path['E']-.1, # error
		 'N'  : path['N']-.1, # error
		 'psi': path['psi'] ,
                 'Ux' : path['v']-.1, # error
                 'Uy' : 0	    ,
		 'r'  : 0
		}				# initialize state
        

        sim = X1Simulator() # initialize sim
        sim.setState(state) # initialize sim state
        #while(1):
        E = []
        N = []
        UX = []
        for i in range(1000):
            state = simLoop()				# run simulation loop
            E = np.r_[E, state['E']]
            N = np.r_[N, state['N']]
            UX = np.r_[UX,state['Ux']]

        plt.subplot(211)
        plt.plot(E,N)
        plt.axis('equal')
        plt.subplot(212)
        plt.plot(UX)
        plt.tight_layout()
        plt.show()