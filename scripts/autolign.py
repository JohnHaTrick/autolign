#!/usr/bin/env python

import sys, time, math
import numpy             as np
import ctrl_autolign     as ctrl
import learn_autolign    as learn
import matplotlib.pyplot as plt
from   sim_autolign  import X1Simulator
try: 
    import ros_autolign
except:
    print "\nROS not available. Simulation only."

# High-level script for the automatic alignment of X1's wheels
#   README to get started

def simLoop():
    state = sim.getState()
    print "E:",round(state['E'],2)," N:",round(state['N'],2)," Psi:",round(state['psi'],2),\
          " Ux:",round(state['Ux'],2)," Uy:",round(state['Uy'],2)," r:",round(state['r'],2)
    del_cmd  = ctrl.lookAheadCtrl(path,state)	# calc steer cmds
    del_cmd += misalign # - guess		# apply misalignment guess
    Fxr      = ctrl.PI_Ctrl(path,state)		# calc longitudinal cmd (rear axle)
    print "Command %.2f rad steering and %.2f N throttle" % (float(del_cmd[0]),Fxr)
    state = sim.simulateX1(del_cmd, Fxr, dt)	# simulate
    print learn.gradientDescent() + '\n'	# calc RL
    #time.sleep(.1)
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
    dt    = .01					#   100 msec per trial
    T     =  10					#   10 sec experiment / simulation
    N     = int(T/dt)				#   number of trials
    guess = [[np.nan for x in range(4)] \
	             for y in range(N)]		#   init misalignment guesses
    guess[0][:] = [0,0,0,0]			#     to zero [FL, FR, RL, RR]
    #path  = ctrl.loadPath_debug()		#   get path
    path  = ctrl.loadPath_VAIL()		#   get path

    if mode == 'exp':				# Mode == exp
        ros_autolign.listener()			#   collect initial state
        while(1):
            expLoop()				#   run experiment loop

    elif mode == 'sim':
        misalign = 5*np.random.uniform(-1,1,4) \
                 * math.pi/180			# [rad] init true misalignment
	print "X1's simulated initial misalignment is: " + str(misalign) + ' radians\n'
        state = {'E'  : path['E']+1, # error
		 'N'  : path['N']  , # error
		 'psi': path['psi']+.2,
                 'Ux' : path['v']+1, # error
                 'Uy' : 0	   ,
		 'r'  : 0
		}				# initialize state
        

        sim = X1Simulator() # initialize sim
        sim.setState(state) # initialize sim state
        E = []
        N = []
        UX = []
        for i in range(int(T/dt)):
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
