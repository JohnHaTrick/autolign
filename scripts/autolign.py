#!/usr/bin/env python

import sys, time, math
import numpy             as np
import ctrl_autolign     as ctrl
import learn_autolign    as learn
import matplotlib.pyplot as plt
import fileman           as fm
from   sim_autolign  import X1Simulator
try: 
    import ros_autolign
except:
    print ("\nROS not available. Simulation only.")

# High-level script for the automatic alignment of X1's wheels
#   README to get started

def simLoop(guess,i):
    state    = sim.getState()				# current state
    e = ctrl.calcLateralError(path,state)
    v = state['Ux']
    print "i: ",i," E:",round(state['E'],2)," N:",round(state['N'],2)," Psi:",round(state['psi'],2),\
          " Ux:",round(state['Ux'],2)," Uy:",round(state['Uy'],2)," r:",round(state['r'],2)
    del_cmd  = ctrl.lookAheadCtrl(path,state)	        # calc steer cmds
    #del_cmd -= guess					# apply misalignment and guess
    Fxr      = ctrl.PI_Ctrl(path,state)		# calc longitudinal cmd (rear axle)
    print "Steer %.2f rad and throttle %.2f N" % (float(del_cmd[0]),Fxr)
    del_real = del_cmd + misalign
    state    = sim.simulateX1(del_real, Fxr, dt)	# simulate cmds
    guess    = learn.gradDescent(guess, del_cmd,
                                 state, dt, path, i)	# calc RL
    print "Tru mis: [%.3f,%.3f,%.3f,%.3f]\n"%(misalign[0],misalign[1],misalign[2],misalign[3])\
	+ "Guess:   [%.3f,%.3f,%.3f,%.3f]\n"%(guess[0],guess[1],guess[2],guess[3]) 

    fm.write_row_csv( filename,
      {'mis1':misalign[0],'mis2':misalign[1],'mis3':misalign[2],'mis4':misalign[3],
        'guess1':guess[0], 'guess2':guess[1], 'guess3':guess[2], 'guess4':guess[3],
       'delta':del_cmd[0],         'Fxr':Fxr,             'e':e,             'v':v},
                      order=ordered )
    #time.sleep(.01)
    return state,np.stack(guess)

def expLoop():
    # calc cmds
    # apply misalignment guess
    # send to X1 to execute
    # receive X1 state update
    # calc RL
    return -1

def welcome(args):
    print ("\nStarting %s" % args[0][-11:-3])   # Welcome message
    try:                                        # Did user enter a terminal argument?
        mode = args[1]              # choose simulate or experiment mode
        if mode == 'sim' or mode == 'exp':
            print ("in %s mode\n" % args[1])
        else:
            print ("in sim mode (by default)\n")
            mode = 'sim'
    except:
        print ("in sim mode (by default)\n")
        mode = 'sim'
    return mode
    
if __name__ == '__main__':			# main function
						# Parameters and initializations:
    mode  = welcome(sys.argv)			#   choose simulation or experiment
    dt    = .01					#   100 msec per trial
    T     =  4					#   10 sec experiment / simulation
    n     = int(T/dt)				#   number of trials
    guess = np.array([0,0,0,0])			#     to zero [FL, FR, RL, RR]
    path  = ctrl.loadPath_debug()		#   get path
    #path  = ctrl.loadPath_VAIL()		#   get path

    if mode == 'exp':               # Mode == exp
        ros_autolign.listener()         #   collect initial state
        while(1):
            expLoop()               #   run experiment loop

    elif mode == 'sim':
        misalign = 5*np.random.uniform(-1,1,4) \
                 * math.pi/180			# [rad] init true misalignment
	print "X1's simulated initial misalignment is: " + str(misalign) + ' radians\n'
        state_0 = {'E'  : path['E']+0, # error
		   'N'  : path['N']+0, # error
		   'psi': path['psi'],
                   'Ux' : path['v']  , # error
                   'Uy' : 0	     ,
		   'r'  : 0
		  }				# initialize state
        
        sim = X1Simulator() # initialize sim
        sim.setState(state_0) # initialize sim state
        E = []
        N = []
        UX = []

	# create file
	#ordered = ['misalign1','misalign2','misalign3','misalign4']
	ordered = [  'mis1',     'mis2',     'mis3',     'mis4',
                   'guess1',   'guess2',   'guess3',   'guess4',
                    'delta',      'Fxr',        'e',        'v']
	filename   = fm.initializer(order=ordered)
	start_time = time.time()
	
        for i in range(n):
            state,guess = simLoop(guess,i)	# run simulation loop
            E = np.r_[E, state['E']]
            N = np.r_[N, state['N']]
            UX = np.r_[UX,state['Ux']]
	
	#sim = X1Simulator()
	ctrl.Integrator.i = 0
	sim.setState(state_0)
	for j in range(n):
	    state    = sim.getState()				# current state
	    e = ctrl.calcLateralError(path,state)
	    v = state['Ux']
	    print "j: ",j,"E:",round(state['E'],2)," N:",round(state['N'],2), \
                  " Psi:",round(state['psi'],2)," Ux:",round(state['Ux'],2), \
                  " Uy:",round(state['Uy'],2)," r:",round(state['r'],2)
	    del_cmd  = ctrl.lookAheadCtrl(path,state)	        # calc steer cmds
	    del_cmd -= guess					# apply misalignment and guess
	    Fxr      = ctrl.PI_Ctrl(path,state)		# calc longitudinal cmd (rear axle)
	    print "Steer %.2f rad and throttle %.2f N" % (float(del_cmd[0]),Fxr)
	    del_real = del_cmd + misalign
	    state    = sim.simulateX1(del_real, Fxr, dt)	# simulate cmds
	    print "Tru mis: [%.3f,%.3f,%.3f,%.3f]\n"%(misalign[0],misalign[1],misalign[2],misalign[3])\
		+ "Guess:   [%.3f,%.3f,%.3f,%.3f]\n"%(guess[0],guess[1],guess[2],guess[3]) 

	    fm.write_row_csv( filename,
              {'mis1':misalign[0],'mis2':misalign[1],'mis3':misalign[2],'mis4':misalign[3],
	        'guess1':guess[0], 'guess2':guess[1], 'guess3':guess[2], 'guess4':guess[3],
	       'delta':del_cmd[0],         'Fxr':Fxr,             'e':e,             'v':v},
	                      order=ordered )
	    	    
        plt.subplot(211)
        plt.plot(E,N)
        plt.axis('equal')
        plt.subplot(212)
        plt.plot(UX)
        plt.tight_layout()
        plt.show()
