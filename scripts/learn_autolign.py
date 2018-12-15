#!/usr/bin/env/ python

from   math              import sin, cos, atan2, pi
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

    NLSim = NonlinearSimulator_2()			# calc next state w rand guess
    delta_rand = np.array(delta) + np.array(guess_rand)
    state_1_model = NLSim.simulate_T( state_0, delta_rand, Fxr, dt )		

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
	
	w = np.linalg.lstsq(PHI,v)[0]
   
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

def valueSearch(guess, delta, Fxr, state_1, dt, iteration, current_real):
    # 0. Initializations:
    #    state_0 & window size
    # 1. Setup new iteration:
    #    define descretizations
    # 2. Search:
    #	 Calculate values for each combination of candidates
    #    best guess :=  best combination
    # 3. Cleanup:
    #    send new best guess
    #    Decrease window size by 10%


    # 0. Initializations:
    bins    =  5				# number of bins. Must be odd

    if not memory.state_0:			# First iter -> initialize params & values:
	print 'No value search for 1st iteration'
	memory.state_0 = state_1		#   save state for next time
	memory.window  = 5*pi/180		#   search field either side
	return guess				#   First iter done, no learning
    state_0        = memory.state_0
    memory.state_0 = state_1			# set state_0 for next iter
    window         = memory.window    
    print 'window = ',round(window,4)
 
    # 1. Setup:
    guesses = dict()				# what angles are we dealing with?
    for i in range(bins):			# FR
      #angle_1 = i * (2*window/(bins-1)) - window + guess[0]
      angle_1 = window*np.power(2.0/(bins-1),3)*np.power(i+(1-bins)/2.0,3)+guess[0]
      for j in range(bins):			# FL
	#angle_2 = j * (2*window/(bins-1)) - window + guess[1]
        angle_2 = window*np.power(2.0/(bins-1),3)*np.power(j+(1-bins)/2.0,3)+guess[1]
	for k in range(bins):			# RR
	  #angle_3 = k * (2*window/(bins-1)) - window + guess[2]
          angle_3 = window*np.power(2.0/(bins-1),3)*np.power(k+(1-bins)/2.0,3)+guess[2]
	  for l in range(bins):			# RL
	    #angle_4 = l * (2*window/(bins-1)) - window + guess[3]
            angle_4 = window*np.power(2.0/(bins-1),3)*np.power(l+(1-bins)/2.0,3)+guess[3]

	    guesses[(i,j,k,l)] = (angle_1,angle_2,angle_3,angle_4)
	    #guesses[(i+bins,j+bins,k+bins,l+bins)] = (angle_2,angle_1,angle_4,angle_3)
	    		    
	    #print 'guesses for ',(i,j,k,l),' are ',roundTupl((angle_1,angle_2,angle_3,angle_4),4) 

    # 2. Search
    NLSim       = NonlinearSimulator_2()	# initialize simulator
    best_value  = -float('inf')			# store best value here
    best_guess  = ()				# store best guess bin here
    for key in guesses.keys():			# try each combination of candidate guesses
	guess_try = guesses[key]		#   try these guesses
	delta_try = np.array(delta) \
                  + np.array(guess_try)		#   add to commanded delta and simulate
	s_1_model = NLSim.simulate_T( state_0, delta_try, Fxr, dt )
	value     = -calcCost(state_1,s_1_model)#   calculate cost
	'''
	current_try  = steeringCurrent(delta_try)      # add current term to cost
	current_diff = np.add(current_real,-1*current_try)
	value       -= np.linalg.norm(current_diff,2)
	'''
	#value -= np.abs(np.sum(guess_try))

	#print 'bins = ',roundTupl(guess_try,4),' value value: ',round(value,6)
	if value > best_value:
	    #print 'Found new best! bins = ',roundTupl(guess_try,4),' with value: ',round(value,6)
	    best_value = value
	    best_guess = guess_try		# after this loop, we've found the best bin
    '''
    # Hmmm, something's not right...
    correct_guess = (.011,0,0,0)
    correct_delta = np.array(delta)+np.array(correct_guess)
    s_1_model = NLSim.simulate_T( state_0, correct_delta, Fxr, dt )		
    correct_value = -calcCost(state_1,s_1_model)
    print 'value for correct guess of ',correct_guess,' = ',correct_value

    nearest_guess = (.01,0,0,0)
    nearest_delta = np.array(delta)+np.array(nearest_guess)
    s_1_model = NLSim.simulate_T( state_0, nearest_delta, Fxr, dt )		
    nearest_value = -calcCost(state_1,s_1_model)
    print 'value for nearest guess of ',nearest_guess,' = ',nearest_value
    '''

    memory.window = window * 0.99		# decrease window size
    return list(best_guess)			# return new best guess

def calcCost(s_1,s_2):		# calculate positive-valued cost
				#    between full-states
    weight        = dict()
    weight['E']   = 0	    	# state variable weights
    weight['N']   = 0	
    weight['psi'] = 0
    weight['r']   = 1
    weight['Ux']  = 1
    weight['Uy']  = 1

    cost = 0
    for key in weight.keys():	# sum of squares
	cost += np.power( (s_1[key] - s_2[key]) , 2 ) * weight[key]
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

    delpg = delta + guess

    Ux = s_0['Ux']
    Uy = s_0['Uy']
    r  = s_0['r']

    NLSim = NonlinearSimulator()		# instantiate nonLinSim
    x1    = X1()				# X1 parameters
    Fx    = [ 0, 0, Fxr, Fxr ]			# package Fx
    deriv = NLSim.getDerivative(s_0,Fx,delpg)	# calc derivatives
    s_0   = dict2Vect(s_0)			# convert dictionaries
    s_1   = dict2Vect(s_1)			#   to vectors

    # Fy_i =  - C_i  * alpha_i                  ### C values are negative ###
    Fy     = []
    # Fy_1 =  - C_1  * ( -delta_1  + atan (  Uy + ar / Ux - dr  ) )
    Fy.append(+x1.C1 * ( -delpg[0] + atan2( Uy+x1.a*r,Ux-x1.d*r ) ) )
    # Fy_2 =  - C_2  * ( -delta_2  + atan(   Uy + ar / Ux + dr  ) )
    Fy.append(+x1.C2 * ( -delpg[1] + atan2( Uy+x1.a*r,Ux+x1.d*r ) ) )
    # Fy_3 =  - C_3  * ( -delta_3  + atan(   Uy - br / Ux - dr  ) )
    Fy.append(+x1.C3 * ( -delpg[2] + atan2( Uy-x1.b*r,Ux-x1.d*r ) ) )
    # Fy_4 =  - C_4  * ( -delta_4  + atan(   Uy - br / Ux + dr  ) )
    Fy.append(+x1.C4 * ( -delpg[3] + atan2( Uy-x1.b*r,Ux+x1.d*r ) ) )

    ddx_dd1 = np.array([  0,  0,  0,
			(-Fx[0]*sin(delpg[0])
			 +x1.C1*sin(delpg[0])
			 -Fy[0]*cos(delpg[0]) )/x1.M,
			( Fx[0]*cos(delpg[0])
			 -x1.C1*cos(delpg[0])
                         -Fy[0]*sin(delpg[0]) )/x1.M,
		 ( x1.a*( Fx[0]*cos(delpg[0])
                         -x1.C1*cos(delpg[0])
                         -Fy[0]*sin(delpg[0]) )
		  +x1.d*( Fx[0]*sin(delpg[0])
                         -x1.C1*sin(delpg[0])
                         +Fy[0]*cos(delpg[0]) ) )/x1.Iz
			])			# partial of dx wrt delta-1
    ddx_dd2 = np.array([  0,  0,  0,
			(-Fx[1]*sin(delpg[1])
			 +x1.C2*sin(delpg[1])
			 -Fy[1]*cos(delpg[1]) )/x1.M,
			( Fx[1]*cos(delpg[1])
			 -x1.C2*cos(delpg[1])
                         -Fy[1]*sin(delpg[1]) )/x1.M,
		 ( x1.a*( Fx[1]*cos(delpg[1])
                         -x1.C2*cos(delpg[1])
                         -Fy[1]*sin(delpg[1]) )
		  -x1.d*( Fx[1]*sin(delpg[1])
                         -x1.C2*sin(delpg[1])
                         +Fy[1]*cos(delpg[1]) ) )/x1.Iz
			])			# partial of dx wrt delta-2
    ddx_dd3 = np.array([  0,  0,  0,
			(-Fx[2]*sin(delpg[2])
			 +x1.C3*sin(delpg[2])
			 -Fy[2]*cos(delpg[2]) )/x1.M,
			( Fx[2]*cos(delpg[2])
			 -x1.C3*cos(delpg[2])
                         -Fy[2]*sin(delpg[2]) )/x1.M,
		 (-x1.b*( Fx[2]*cos(delpg[2])
                         -x1.C3*cos(delpg[2])
                         -Fy[2]*sin(delpg[2]) )
		  +x1.d*( Fx[2]*sin(delpg[2])
                         -x1.C3*sin(delpg[2])
                         +Fy[2]*cos(delpg[2]) ) )/x1.Iz
			])			# partial of dx wrt delta-3
    ddx_dd4 = np.array([  0,  0,  0,
			(-Fx[3]*sin(delpg[3])
			 +x1.C4*sin(delpg[3])
			 -Fy[3]*cos(delpg[3]) )/x1.M,
			( Fx[3]*cos(delpg[3])
			 -x1.C4*cos(delpg[3])
                         -Fy[3]*sin(delpg[3]) )/x1.M,
		 (-x1.b*( Fx[3]*cos(delpg[3])
                         -x1.C4*cos(delpg[3])
                         -Fy[3]*sin(delpg[3]) )
		  -x1.d*( Fx[3]*sin(delpg[3])
                         -x1.C4*sin(delpg[3])
                         +Fy[3]*cos(delpg[3]) ) )/x1.Iz
			])			# partial of dx wrt delta-4

    ddx_dd  = np.vstack([ddx_dd1,ddx_dd2,ddx_dd3,ddx_dd4])

    # Update rule: guess = guess - eta*grad(J)
    eta = 1/np.sqrt(i/10.0+10)				# learning rate
    
    '''
    print 'ddx_dd shape = ',ddx_dd.T.shape
    print 'ddx_dd = ',ddx_dd.T
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
    window  = float()				# save previous iteration window size

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

def roundTupl(my_tuple,decimals):
    round_list = []
    for val in my_tuple:
	round_list.append(round(val,decimals))
    return tuple(round_list)
