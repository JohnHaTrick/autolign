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

def simLoop(guess):
    state    = sim.getState()				# current state
    del_cmd  = ctrl.lookAheadCtrl(path,state)	        # calc steer cmds
    del_cmd -= guess					# apply misalignment and guess
    Fxr      = 1000*ctrl.PI_Ctrl(path,state)		# calc longitudinal cmd (rear axle)
    print "Steer %.2f rad and throttle %.2f N" % (float(del_cmd[0]),Fxr)
    del_real = del_cmd + misalign
    state    = sim.simulateX1(del_real, Fxr)	# simulate cmds
    guess    = learn.gradDescent(guess, del_cmd,
                                 state, dt, path)	# calc RL
    print "True misalign: [%.3f,%.3f,%.3f,%.3f]\n"%(misalign[0],misalign[1],misalign[2],misalign[3])\
	+ "Guess:         [%.3f,%.3f,%.3f,%.3f]\n"%(guess[0],guess[1],guess[2],guess[3]) 
    time.sleep(.01)
    return state

def expLoop():
    # calc cmds
    # apply misalignment guess
    # send to X1 to execute
    # receive X1 state update
    # calc RL
    return -1

def welcome(args):
    print "\nStarting %s" % args[0][-11:-3]	# Welcome message
    try:                                        # Did user enter a terminal argument?
        mode = args[1]				# choose simulate or experiment mode
        if mode == 'sim' or mode == 'exp':
            print "in %s mode\n" % args[1]
        else:
            print "in sim mode (by default)\n"
            mode = 'sim'
    except:
        print "in sim mode (by default)\n"
        mode = 'sim'
    return mode
    
if __name__ == '__main__':			# main function
						# Parameters and initializations:
    mode  = welcome(sys.argv)			#   choose simulation or experiment
    dt    = .1					#   100 msec per trial
    T     =  1					#   10 sec experiment / simulation
    N     = int(T/dt)				#   number of trials
    #guess = [[np.nan for x in range(4)] \
    #	             for y in range(N)]		#   init misalignment guesses
    #guess[0][:] = [0,0,0,0]			#     to zero [FL, FR, RL, RR]
    guess = np.array([0,0,0,0])			#     to zero [FL, FR, RL, RR]
    path  = ctrl.loadPath_VAIL()		#   get path

    if mode == 'exp':				# Mode == exp
        ros_autolign.listener()			#   collect initial state
        while(1):
            expLoop()				#   run experiment loop

    elif mode == 'sim':
        misalign = 5*np.random.uniform(-1,1,4) \
                 * math.pi/180			# [rad] init true misalignment
	print "X1's simulated initial misalignment is: " + str(misalign) + ' radians\n'
        state = {'E'  : path['E']-.5, # error
		 'N'  : path['N']-.5, # error
		 'psi': path['psi'] ,
                 'Ux' : path['v']-.5, # error
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
            state = simLoop(guess)		# run simulation loop
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
