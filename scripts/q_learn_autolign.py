#!/usr/bin/env/ python
import math
import numpy as np
from sim_autolign import X1Simulator
import ctrl_autolign as ctrl
# import random
# import operator


def calcS1Model(state_1, s_0, s_1, delta, guess, dt, path):
    guess = list(guess)
    guess = np.asarray(guess)
    X1 = X1Simulator()			    # Vehicle parameters:
    a  = X1.a
    b  = X1.b
    M  = X1.M
    Iz = X1.Iz
    Cf = X1.Cf
    Cr = X1.Cr

    memory.state_0 = state_1		    #   set state_0 for next iter
    Ux      = state_1['Ux']		    #   speed (assumed const)
    I       = np.identity(4)		    # Learning parameters
    Q       = (Cr*b   - Cf*a  )/M /Ux
    R	    = -(Cf*a*a + Cr*b*b)/Iz/Ux
    C	    = Cf + Cr
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
    return s_1_model;


def qLearning(delta, state_1, dt, path, i):
    maxMisalign = 1 * math.pi / 180
    minMisalign = -1 * math.pi / 180
    # Constructs grid world states for each wheel
    s1 = np.linspace(minMisalign, maxMisalign, 50)
    s2 = np.linspace(minMisalign, maxMisalign, 50)
    s3 = np.linspace(minMisalign, maxMisalign, 50)
    s4 = np.linspace(minMisalign, maxMisalign, 50)
    # print(s1)
    # Initialize k count to control how many guesses we want to make
    k = 0
    # Initialize guessedStates storing states that has been guessed & evaluated
    guessedStates = set()
    # Initialize guessRewardPair which stores guess and reward pairs
    guessRewardPair = dict()

    if not memory.state_0:		    # First iteration?
        memory.state_0 = state_1	    # Save for next time
        return np.array([0, 0, 0, 0])

    # Repeat the guess and reward assignment step:
    while True:
        # Randomly select a guess from the states to try
        s1Guess = np.random.choice(s1, 1)[0]
        s2Guess = np.random.choice(s2, 1)[0]
        s3Guess = np.random.choice(s3, 1)[0]
        s4Guess = np.random.choice(s4, 1)[0]
        guess = (s1Guess, s2Guess, s3Guess, s4Guess)
        if guess not in guessedStates:
            # Add guessed states to a set
            guessedStates.add(guess)
            # print(guessedStates)
            s_0 = np.array([ctrl.calcLateralError(path, memory.state_0),
                            ctrl.calcHeadingError(path, memory.state_0),
                            memory.state_0['Uy'], memory.state_0['r']])
            # Calculate x_1_meas
            s_1 = np.array([ctrl.calcLateralError(path, state_1),
                            ctrl.calcHeadingError(path, state_1),
                            state_1['Uy'], state_1['r']])
            # Calculate x_1_model
            x_1_model = calcS1Model(state_1, s_0, s_1, delta, guess, dt, path)
            # Calculate Reward = -|Delta x| that is associated with this guess
            reward = -np.absolute(s_1-x_1_model)
            reward = reward.sum()
            # Associate guess with the reward using a dictionary
            guessRewardPair[guess] = reward
            # print(guessRewardPair)
        # increment count and break when ideal number of repetition is reached
        k += 1
        if k == 100:
            break
    # print(guessRewardPair)
    # return guess with highest reward - our best guess of what our state is
    return max(guessRewardPair, key=guessRewardPair.get)
    # input states into Q learning & calculate optimal action under that state?


class memory(object):
    state_0 = dict()
