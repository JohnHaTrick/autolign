#!/usr/bin/env/ python

from   math              import sin, cos, pi
import matplotlib.pyplot as	plt
import random		 as	rand
import numpy		 as     np
from   copy		 import deepcopy
import ctrl_autolign	 as     ctrl
from   sim_autolign	 import NonlinearSimulator,   X1, \
				NonlinearSimulator_2, LinearSimulator

def valueEst(guess, delta, Fxr, state_1, dt, path, iteration):
    # 1: update random guess_values according to the
    #   negative squared difference in measured vs. modelled state:
    #   -(x_1_meas - x_1_model)^2
    # 2: Regress a parabola for each wheel
    # 3: Solve for maximum valued (parabola) guess for each wheel

    # Parameters
    num_bins       = 17			#   Total number of discrete guess bins
    max_angle      = 2 * pi / 180	#   [rad] Check consistency w max misalign

    ## 1:
    if not memory.values:			# First iter -> initialize params & values:
	print 'No value estimation for 1st iteration'
	memory.state_0 = state_1		#   save state for next time
	values         = dict()			#   Init the value table to zero [wheel][angle]
	for i in range(4):
	    values[i+1] = dict()
	for j in range(num_bins):
	    angle = max_angle * ( j - (num_bins-1)/2.0 ) / ((num_bins-1)/2.0)
	    for i in range(4):
		values[i+1][angle] = [0]	#   Each potential guess init to empty list
	memory.values = values			#   save for future use.
	return guess				#   First iter done, no learning
    values         = memory.values		# unpack for this iter
    state_0        = memory.state_0
    memory.state_0 = state_1			# set state_0 for next iter

    # Try assigning only one wheel per iteration
    wheel      = np.mod( iteration, 4 )	+ 1		# working on this wheel in [1 4]
    # wheel = 1
    wheel_rand = rand.sample(values[wheel].keys(),1)[0]	# here's the random assignment

    guess_rand          = list(guess)
    guess_rand[wheel-1] = wheel_rand			# keep the others as best guess so far

    '''
    print 'wheel value keys: ', [ round(e,3) for e in sorted(values[wheel].keys()) ]
    print 'choosing %.3f rad for wheel # %i' % (wheel_rand,wheel)
    print 'guesses for this iteration: 1: %.3f, 2: %.3f, 3: %.3f, 4: %.3f' \
	  % (guess_rand[0],guess_rand[1],guess_rand[2],guess_rand[3])
    '''

    NLSim = NonlinearSimulator_2()			# calc next state w rand guess
    delta_rand = np.array(delta) + np.array(guess_rand)
    state_1_model = NLSim.simulate_T( state_0, delta_rand, Fxr, dt )		

    '''
    print 'state 1        : ',roundDict(deepcopy(state_1)      ,3)
    print 'state 1 w guess: ',roundDict(deepcopy(state_1_model),3)

    '''

    value = -calcCost(state_1,state_1_model)
    #print 'value = ',value

    #for i in range(4):
    values[wheel][guess_rand[wheel-1]].append( value )

    # 2: parabola = w1 + w2*x + w3*x^2 ; phi(x) = [ 1 x x^2 ]
    #      w := PHI(x) \ <v>           ; PHI(x) = [phi(x1);...;phi(xn)]
    #	   <v> = vector of average values for each angle possibility
    
    # Skip unless a non-zero score exists for each angle
    each_angle_tried = 1
    for angle in values[wheel].keys():
	if len(values[wheel][angle]) == 1:
	    each_angle_tried = 0
    if each_angle_tried:

	#print 'all angles have been tried; now try to find optimal'
	
	PHI = np.empty((1,3),float)
	for angle in sorted(values[wheel].keys()):
	    phi = np.array( [[ 1, angle, np.power(angle,2) ]] )
	    PHI = np.append(PHI,phi,0)
	PHI = np.delete(PHI,(0),axis=0)		# remove empty first row
	#print "PHI: ", PHI
	    
	v = []
	for angle in sorted(values[wheel].keys()):
	    v.append( np.mean( values[wheel][angle] ) )
	v = np.array(v)
	#print 'average values: ',v
	
	w = np.linalg.lstsq(PHI,v)[0]
	#print 'w = ',w_1
   
	# 3: derivative of parabola wrt x = 0
	guess_wheel = -w[1] / w[2] / 2
	guess_wheel = min(guess_wheel, max_angle)
	guess_wheel = max(guess_wheel,-max_angle)

	#print 'updated guess = ',guess_wheel
	guess[wheel-1] = guess_wheel
 
    else:
	print 'All values for wheel %i have not been tried. Go to next iter' % wheel

    #print 'i = ',iteration
    if ( np.mod(iteration,21) == 0 or iteration == 0 ) and each_angle_tried:

	x_deg = []
	x_rad = []
	for angle in sorted(values[1].keys()):
	    x_rad.append(angle)
	    x_deg.append(angle*180/pi)

	y = []
	for i in range(len(x_rad)):
	    y.append(w[0] + w[1]*x_rad[i] + w[2]*x_rad[i]*x_rad[i])

	plt.plot(x_deg,y)
	plt.scatter(x_deg,v)
	plt.show()

    memory.values = values
    return guess

def calcCost(s_1,s_2):		# calculate positive-valued cost
				#    between full-states
    weight        = dict()
    weight['E']   = 1	    	# state variable weights
    weight['N']   = 1	
    weight['psi'] = 1
    weight['r']   = 1
    weight['Ux']  = 10
    weight['Uy']  = 1

    cost = 0
    for key in weight.keys():	# sum of squares
	cost += np.power( (s_1[key] - s_2[key]) * weight[key] ,2 )
    return cost

def gradDescent(guess,delta,state_1,dt,path,i):
    # minimize cost function J = (x_1_meas - x_1_model)^2
    #   using linear model with states [ e dPsi Uy r ]

    if not memory.state_0:		        # First iteration?
	memory.state_0 = state_1	        #   save for next time
	return guess                            #   no GD on 1st iter
    state_0        = memory.state_0		# unpack for this iter
    memory.state_0 = state_1		        # set state_0 for next iter
    
    LinSim = LinearSimulator()		        # Init linear simulator
    s_0    = LinSim.getLocalState(state_0,path) # Local states to path (vector)
    s_1    = LinSim.getLocalState(state_1,path)
    Ux     = state_0['Ux']		        # speed (assumed const)

    s_1_linModel, B_matrix = LinSim.simulate_Lin(s_0,Ux,path,delta+guess,dt)

    eta   = .2/np.sqrt(i)		        # learning rate
    guess = guess + 2 * eta * dt * ( s_1 - s_1_linModel ) * B_matrix
    return guess.tolist()[0]

def gradDescent_NL(guess, delta, Fxr, s_1, dt, i):
    # minimize cost function J = (x_1_meas - x_1_model)^2
    #   using nonlinear model with states [ E N Psi Ux Uy r ]

    if not memory.state_0:		        # First iteration?
	memory.state_0 = s_1		        #   save for next time
	return guess                            #   no GD on 1st iter
    s_0	= memory.state_0			# unpack for this iter
    memory.state_0 = s_1		        # set state_0 for next iter

    NLSim = NonlinearSimulator()		# instantiate nonLinSim
    x1    = X1()				# X1 parameters
    Fx    = [ 0, 0, Fxr, Fxr ]			# package Fx
    deriv = NLSim.getDerivative(s_0,Fx,delta)	# calc derivatives
    s_0   = dict2Vect(s_0)			# convert dictionaries
    s_1   = dict2Vect(s_1)			#   to vectors

    ddx_dd1 = np.array([  0,  0,  0,
			(-Fx[0]*sin(delta[0])
			 +x1.C1*sin(delta[0])
			 +x1.C1*cos(delta[0])*delta[0])/x1.M,
			( Fx[0]*cos(delta[0])
			 -x1.C1*cos(delta[0])
                         +x1.C1*sin(delta[0])*delta[0])/x1.M,
		 ( x1.a*( Fx[0]*cos(delta[0])
                         -x1.C1*cos(delta[0])
                         +x1.C1*sin(delta[0])*delta[0] )
		  +x1.d*( Fx[0]*sin(delta[0])
                         -x1.C1*sin(delta[0])
                         -x1.C1*cos(delta[0])*delta[0]))/x1.Iz
			])			# partial of dx wrt delta-1
    ddx_dd2 = np.array([  0,  0,  0,
			(-Fx[1]*sin(delta[1])
			 +x1.C2*sin(delta[1])
			 +x1.C2*cos(delta[1])*delta[1])/x1.M,
			( Fx[1]*cos(delta[1])
			 -x1.C2*cos(delta[1])
                         +x1.C2*sin(delta[1])*delta[1])/x1.M,
		 ( x1.a*( Fx[1]*cos(delta[1])
                         -x1.C2*cos(delta[1])
                         +x1.C2*sin(delta[1])*delta[1] )
		  -x1.d*( Fx[1]*sin(delta[1])
                         -x1.C2*sin(delta[1])
                         -x1.C2*cos(delta[1])*delta[1]))/x1.Iz
			])			# partial of dx wrt delta-2
    ddx_dd3 = np.array([  0,  0,  0,
			(-Fx[2]*sin(delta[2])
			 +x1.C3*sin(delta[2])
			 +x1.C3*cos(delta[2])*delta[2])/x1.M,
			( Fx[2]*cos(delta[2])
			 -x1.C3*cos(delta[2])
                         +x1.C3*sin(delta[2])*delta[2])/x1.M,
		 (-x1.b*( Fx[2]*cos(delta[2])
                         -x1.C3*cos(delta[2])
                         +x1.C3*sin(delta[2])*delta[2] )
		  +x1.d*( Fx[2]*sin(delta[2])
                         -x1.C3*sin(delta[2])
                         -x1.C3*cos(delta[2])*delta[2]))/x1.Iz
			])			# partial of dx wrt delta-3
    ddx_dd4 = np.array([  0,  0,  0,
			(-Fx[3]*sin(delta[3])
			 +x1.C4*sin(delta[3])
			 +x1.C4*cos(delta[3])*delta[3])/x1.M,
			( Fx[3]*cos(delta[3])
			 -x1.C4*cos(delta[3])
                         +x1.C4*sin(delta[3])*delta[3])/x1.M,
		 (-x1.b*( Fx[3]*cos(delta[3])
                         -x1.C4*cos(delta[3])
                         +x1.C4*sin(delta[3])*delta[3] )
		  -x1.d*( Fx[3]*sin(delta[3])
                         -x1.C4*sin(delta[3])
                         -x1.C4*cos(delta[3])*delta[3]))/x1.Iz
			])			# partial of dx wrt delta-4
    ddx_dd  = np.vstack([ddx_dd1,ddx_dd2,ddx_dd3,ddx_dd4])

    # Update rule: guess = guess - eta*grad(J)
    eta = 1/np.sqrt(i)				# learning rate
    '''
    print '\n\nguess = \n',guess
    print 'eta = ',eta
    print 'dt = ',dt
    print 'ved_diff = ',vec_add([s_1,-s_0,-dt*deriv])
    print 'ddx_dd = ',ddx_dd
    '''
    guess = guess + 2 * eta * dt * vec_add([s_1,-s_0,-dt*deriv]) * ( ddx_dd.T )
    return guess.tolist()[0]

## HELPER FUNCTIONS ##
class memory(object):
    state_0 = dict()				# save last state here
    values  = dict()				# save value list here [ 4 x num_bins ]

def roundDict(dictionary,decimals):		# Round each value in the dict
    for key in dictionary.keys():		#   CAREFUL! arg will be modified!
	dictionary[key] = round(dictionary[key],decimals)
    return dictionary

def dict2Vect(s_dict):				# [ E N Psi Ux Uy r ]
    return np.array([s_dict['E'], s_dict['N'], s_dict['psi'],
		     s_dict['Ux'],s_dict['Uy'],s_dict['r']   ])

def vec_add(vec_list):
    vec_sum = vec_list[0]
    for vec in vec_list[1:]:
	vec_sum = np.add(vec,vec_sum)
    return vec_sum

