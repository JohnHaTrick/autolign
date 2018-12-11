#!/usr/bin/env python

import numpy        as     np
from   sim_autolign import ackermann

def pathTrackingCmds(path,state):
    delta = lookAheadCtrl(path,state)
    Fxr   = PI_Ctrl(path,state)
    return  delta, Fxr

def lookAheadCtrl(path,state):
    # Kapania & Gerdes. Autonomous Lanekeeping Sys for Path Tracking. AVEC. 2014
    del_max = .3
    x_LA    =  20				# [m]     look ahead dist
    k_p	    =  .2				# [rad/m] proportional gain
    e       =  calcLateralError(path,state)	# [m]     current lateral error
    dPsi    =  calcHeadingError(path,state)     # [rad]   heading error
    e_LA    =  e + x_LA * dPsi			# [m]     look ahead error
    #print "e: ",round(e,2)," e_LA: ",round(e_LA,2)," dPsi: ",round(dPsi,2)
    del_FB  = -k_p * e_LA			# [rad]   FB (no sideslip)
    del_FFW =  4*path['k'] - 0 + 0		# [rad]   FFW (not implemented)
    del_f   =  del_FFW + del_FB
    if del_f > del_max:	   del_f = del_max      # assert maximum steering angle
    elif del_f < -del_max: del_f = -del_max
    del_r   =  0
    delta   =  ackermann( del_f, del_r )        # Ackerman correction needed?
    return delta		                # [FL, FR, RL, RR]

def PI_Ctrl(path,state):
    v_error = path['v'] - state['Ux']		# [m/s]   v error (if Ux >> Uy)
    k_p     = 2000				# [N/mps] proportional gain
    k_i     = 20				# [N/mps] integral gain
    try:    Integrator.i
    except: Integrator()			# instantiate integraor
    Integrator.i += v_error			# integrate
    return k_p * v_error + k_i * Integrator.i	# [N] cruise control throttle

class Integrator(object):
    i = 0

# return path information
def loadPath_debug():
    path = {'name' : 'debug'  ,
            'E'    :    0     ,			# [m]   init pos East
            'N'    :    0     ,			# [m]   init pos North
            'psi'  :    0     ,			# [rad] init heading, ccw from N
            'k'    :    0     ,			# [1/m] curvature
            'v'    :    4      }		# [m/s] desired speed
    return path

def loadPath_VAIL():
    path = {'name' : 'VAIL'   ,
            'E'    :   22.1068,			# [m]   init pos East. Durand Datum
            'N'    :  -10.7145,			# [m]   init pos North
            'psi'  :    1.520 ,			# [rad] init heading, ccw from N
            'k'    :    0     ,			# [1/m] curvature
            'v'    :    4      }		# [m/s] desired speed
    return path

def loadPath_Fremont():
    path = {'name' : 'Fremont',
            'E'    :   53.3943,			# [m]   init pos East. Durand Datum
            'N'    : -155.6835,			# [m]   init pos North
            'psi'  :    2.6726,			# [rad] init heading, ccw from N
            'k'    :    0     ,		    	# [1/m] curvature
            'v'    :    5      }		# [m/s] desired speed
    return path

def calcLateralError(path,state):
    # signed distance left of the straight path
    s_hat = np.array( [-np.sin(path['psi']),	# unit vec in path direction
                        np.cos(path['psi'])]  )	#  -sin(psi), cos(psi)
    e_hat = np.array( [-np.cos(path['psi']),	# unit vec in lateral e dir
                       -np.sin(path['psi'])]  )	#  -sin(psi+pi/2), cos(psi+pi/2)
    d_vec = np.array( [ state['E']-path['E'],	# from init to current position
                        state['N']-path['N']] )	#   (pos.E-path.E, pos.N-path.N)
    s_vec = np.dot(s_hat,d_vec) * s_hat		# from init to cloest pt on path
    e_vec = d_vec - s_vec			# from closest pt to current pos
    e     = np.dot(e_hat,e_vec)			#    lateral error
    return e

def calcHeadingError(path,state):
    return state['psi'] - path['psi']
