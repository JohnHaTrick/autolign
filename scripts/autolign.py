#!/usr/bin/env python

import sys, time, math
import numpy             as np
import ctrl_autolign     as ctrl
import learn_autolign    as learn
import matplotlib.pyplot as plt
import fileman           as fm
import sim_autolign      as sim
import util_autolign     as util
from   plot_autolign import Plotter
try: 
    import ros_autolign
except:
    print ("\nROS not available. Simulation only.")

# High-level script for the automatic alignment of X1's wheels
#   README to get started

def simLoop(guess,i,sim_mode):
    state    = NLsim.getState()				# current state
    e        = ctrl.calcLateralError(path,state)
    v        = state['Ux']
    print "i: ",i," E: "  ,round(state['E'],2)  ," N: " ,round(state['N'],2), \
                  " Psi: ",round(state['psi'],2)," Ux: ",round(state['Ux'],2),\
                  " Uy: " ,round(state['Uy'],2) ," r: " ,round(state['r'],2)
    del_cmd  = ctrl.lookAheadCtrl(path,state)	        # calc steer cmds
    del_cmd += .2 * np.sin( i / 10 )
    if sim_mode == 'val':                               # if validtating,
        del_cmd -= guess      	                        #   apply guess
    Fxr      = ctrl.PI_Ctrl(path,state)			# calculate Fx cmd
    print "Steer %.2f rad and throttle %.2f N\n" % (float(del_cmd[0]),Fxr)
    del_real = del_cmd + misalign                       # apply misalignment
    state    = NLsim.simulate_T(del_real, Fxr, dt)	# simulate cmds
    if sim_mode == 'lrn':                               # if learning,
        #guess = learn.gradDescent(guess, del_cmd,
        #                          state, dt, path, i)	#   calc gradDescent
	guess = learn.valueEst(guess, del_cmd, Fxr,
			       state, dt, path, i)	#   or calc value estimation
    e = ctrl.calcLateralError(path,state)               # update storage
    dPsi = ctrl.calcHeadingError(path,state)            #   for plotting
    state.update(delta=del_cmd, guess=guess, misalign=misalign, e=e, dPsi=dPsi)
    print "\nTru mis: [%.3f,%.3f,%.3f,%.3f]\n"                \
        % (misalign[0],misalign[1],misalign[2],misalign[3]) \
	+ "Guess:   [%.3f,%.3f,%.3f,%.3f]\n\n"              \
        % (guess[0],guess[1],guess[2],guess[3])         # status update
    fm.write_row_csv( filename, {'mis1':misalign[0], 'mis2':misalign[1],
                                 'mis3':misalign[2], 'mis4':misalign[3],
                                 'guess1':guess[0],  'guess2':guess[1] ,
                                 'guess3':guess[2],  'guess4':guess[3] ,
                                 'delta':del_cmd[0], 'Fxr':Fxr         ,
                                 'e':e,              'v':v             },
                      order=ordered )                   # write data
    state.update(t=dt*i)                                # add time to dict
    plotter.store(state)                                # store for plotting
    #time.sleep(.01)                                    # pause for debug?
    return np.stack(guess)                              # end learning loop

def expLoop():
    # calc cmds
    # apply misalignment guess
    # send to X1 to execute
    # receive X1 state update
    # calc RL
    return -1
    
if __name__ == '__main__':			    #### main function ####
						    # Params and inits:
    mode  = util.welcome(sys.argv)      	    #   choose sim or exp
    dt    = .01					    #   100 msec per trial
    T     = 10 					    #   [s] sim/exp term
    n     = int(T/dt)				    #   num trials
    guess = np.array([0.0,0.0,0.0,0.0])		    #   [FL, FR, RL, RR]
    path  = ctrl.loadPath_debug()		    #   get path
    #path  = ctrl.loadPath_VAIL()		    #   get path

    start_time = time.time()		            # init save file using
    ordered = [ 'mis1'  , 'mis2'  , 'mis3'  , 'mis4'  ,
                'guess1', 'guess2', 'guess3', 'guess4',
                'delta' , 'Fxr'   , 'e'     , 'v'      ]
    filename   = fm.initializer(order=ordered)      #   this header & timestamp

    plotter = Plotter()                             # init Plotter class

    if mode == 'exp':				    ### Mode == exp ###
        ros_autolign.listener()			    # collect init state
        while(1): expLoop()			    # run exp loop

    elif mode == 'sim':                             ### Mode == sim ###
        misalign = 2 * np.random.uniform(-1,1,4) \
                     * math.pi/180		    # [rad] set true alignment
	misalign = .5 * np.array([ 1, 0, 0, 0 ]) \
		     * math.pi/180		    # debug
        state_0 = {'E'  : path['E']+0, # error?
		   'N'  : path['N']+0, # error?
		   'psi': path['psi'],
                   'Ux' : path['v']  , # error?
                   'Uy' : 0	     ,
		   'r'  : 0           }		    # initialize state dict()
        NLsim   = sim.NonlinearSimulator()	    # init NL sim class

                                                    ## Learning Trials ##
        NLsim.setState(state_0)			    # init NL sim state
        for i in range(1,n+1):                      # n learning iterations
            guess = simLoop(guess,i,'lrn')          # sim loop : learning mode
	
        '''                                            ## Validation Trials ##
	NLsim.setState(state_0)			    # Reset sim state
	ctrl.Integrator.i = 0			    # Reset ctrl integrator
	for j in range(1,n+1):                          # n validation iterations
            guess = simLoop(guess,j,'val')          # sim loop : validation mode
	'''    
        plotter.plot()                              # plot simulation result
