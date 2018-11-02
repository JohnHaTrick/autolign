#!/usr/bin/env python

import sys
import numpy as np
import ros_autolign
import ctrl_autolign
import sim_autolign
import learn_autolign

# Two modes: exp and sim
#   exp: listens to from_autobox, calc autolign, then sends to_autobox
#   sim ...

def simLoop():
    # calc cmds
    print ctrl_autolign.lookAheadControl()
    # apply misalignment guess
    # simulate
    print sim_autolign.simulateX1()
    # calc RL
    print learn_autolign.gradientDescent() + '\n'
    return -1

def expLoop():
    # calc cmds
    # apply misalignment guess
    # send to X1 to execute
    # receive X1 state update
    # calc RL
    return -1

if __name__ == '__main__':
    # welcome. Select mode from usr input: 'sim' or 'exp'
    print "\nStarting %s" % sys.argv[0][-11:-3]
    try:
        mode = sys.argv[1]
        if mode == 'sim' or mode == 'exp':
            print "in %s mode\n" % sys.argv[1]
        else:
            print "in sim mode (by default)\n"
            mode = 'sim'
    except:
        print "in sim mode (by default)\n"
        mode = 'sim'

    # some global parameters and initializations
    dt        = .1              # 100 msec per trial
    T         = 1               # 10 sec experiment / simulation
    N         = int(T/dt)       # number of trials
    guess_rad = [[np.nan for x in range(4)] for y in range(N)] # misalignment guesses

    if mode == 'exp':
        # init misalignment guess
        guess_rad[0][:] = [0,0,0,0]
        print guess_rad
        # init path
        # collect intial state
        ros_autolign.listener()
        # run experiment loop
        while(1):
            expLoop()

    elif mode == 'sim':
        # init true misalignment
        # init misalignment guess
        guess_rad[0][:] = [0,0,0,0]
        # init path
        # init state
        # run simulation loop
        while(1):
            simLoop()
