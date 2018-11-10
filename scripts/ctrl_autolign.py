#!/usr/bin/env python

import math
import numpy as np

def lookAheadControl(path,state):
    # Kapania & Gerdes 2014
    x_LA = 10 # [m] look ahead dist
    e    = calcLateralError(path,pos)
    e_LA = e + x_LA * (path['psi'] - state['psi'])
...
    return "Controller: Send path error. Return path-follwing commands."

def PI_Control(path,state):
    v_error = path['v'] - state['v']
    # proportional term
    k_p   = 100           # N per m/s of speed error
    Fxr_p = k_p * v_error
    # integral term
    k_i = 10              # N per m/s of integrated speed error
    i = i + v_error
    Fxr_i = k_i * i
    return Fxr_p + Fxr_i
    

# return path information
def loadPath_VAIL():
    path = {'name' : 'VAIL'
            'E'    :   22.1068, \ # [m]   init pos East. Durand Datum
            'N'    :  -10.7145, \ # [m]   init pos North
            'psi'  :    1.520 , \ # [rad] init heading
            'k'    :    0     , \ # [1/m] curvature
            'v'    :    3     , \ # [m/s] desired speed
    return path

def loadPath_Fremont():
    path = {'name' : 'Fremont'
            'E'    :   53.3943, \ # [m]   init pos East. Durand Datum
            'N'    : -155.6835, \ # [m]   init pos North
            'psi'  :    2.6726, \ # [rad] init heading
            'k'    :    0     , \ # [1/m] curvature
            'v'    :    5     , \ # [m/s] desired speed
    return path

def calcLateralError(path,pos):
    # signed distance left of the straight path
    s_hat = np.array([-np.sin(path['psi']), \ # unit vec in path direction
                       np.cos(path['psi'])])  #   (-sin(psi), cos(psi)
    e_hat = np.array([ np.cos(path['psi']), \ # unit vec in lateral error direction
                       np.sin(path['psi'])])  #   (-sin(psi+pi/2), cos(psi+pi/2))
    d_vec = np.array([ pos['E']-path['E'] , \ # from init to current position
                       pos['N']-path['N']] )  #   (pos.E-path.E, pos.N-path.N)
    s_vec = np.dot(s_hat,d_vec) * s_hat       # from init to cloest pt on path
    e_vec = d_vec - s_vec                     # from closest pt to current position
    e     = e_hat * e_vec                     # lateral error
    return e
