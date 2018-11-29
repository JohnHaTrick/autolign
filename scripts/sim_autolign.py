#!/usr/bin/env python

import numpy as np
import ctrl_autolign as ctrl

# sim_autolign provides simulation models and parameters for X1

def ackermann(delta_f,delta_r):
    # Linear model of ackermann angle
    # Small angle approx : delta_f,r,fl,fr,rl,rr << 1
    #    allows tan, tan-1 ==> 1. 
    # Input
    #   d: half length of tread
    #   l: wheel base
    X1()
    x        = X1.d / X1.l * ((delta_f) - (delta_r))
    delta_fl = delta_f / (1 - x)
    delta_fr = delta_f / (1 + x) 
    delta_rl = delta_r / (1 - x) 
    delta_rr = delta_r / (1 + x)
    return [ delta_fl, delta_fr, delta_rl, delta_rr ]

class X1(object):
    """ X1 Vehicle Parameter Class"""
    M  =    1964.0                  # [kg]     Mass
    a  =       1.50                 # [m]      Front length
    b  =       1.37                 # [m]      Rear length  
    l  =  a + b                     # [m]      Wheel base
    d  =       0.82                 # [m]      Half Tread
    Iz =    2900.0                  # [kg-m^2] Inertia 2250 = 1000*1.5*1.5
    Cf = -140000.0                  # [N/rad]  Front Axle Cornering Stiffness 
    Cr = -190000.0                  # [N/rad]  Rear Axle Cornering Stiffness
    C1 =  Cf / 2.0                  # [N/rad]  Front Left Cornering Stiffness
    C2 =  Cf / 2.0                  # [N/rad]  Front Right Cornering Stiffness
    C3 =  Cr / 2.0                  # [N/rad]  Rear Left Cornering Stiffness
    C4 =  Cr / 2.0                  # [N/rad]  Rear Right Cornering Stiffness
    C  =  Cf + Cr                   # [N/rad]  Total Cornering Stiffness

class NonlinearSimulator:
    """ High Fidelity 4 Wheel Vehicle Simulation Class """

    def __init__(self):
        X1()                        #     Vehicle params
        self.dt       = 0.001	    # [s] Internal sim time step
        self.east     = 0.0         #     Simulation-level states
        self.north    = 0.0
        self.psi      = 0.0
        self.vx       = 0.0
        self.vy       = 0.0
        self.yawrate  = 0.0
        self.fx       = [0.0] * 4   #     Simulation commands
        self.delta_fr = [0.0] * 2
    
    def setStateValue(self,psi,east,north,r,vx,vy):
        self.psi   = psi
        self.east  = east
        self.north = north
        self.r     = r
        self.vx    = vx
        self.vy    = vy
            
    def setCommand(self,fx,delta_fr):
        # fx:       List of 4 longitudinal forces to each wheel(FL,FR,RL,RR)
        # delta_fr: List of 2 steering angles to each axles(Front, Rear)
        self.fx = fx
        self.delta_fr = delta_fr

    def simulate_dt(self):
        # propogate self.state forward by one (small) dt
        #   Not the same as the (larger) autolignment dt

        # Calc ackermann steer angle of 4 wheels, in case of bicycle commands
        # self.delta_4ws = ackermann(self.del_fr, self.del_fr, self.d, self.l)
        [delta1, delta2, delta3, delta4] = self.delta_4ws
        [fx1, fx2, fx3, fx4]             = self.fx
        
        # Calc forces to each tire    
        fy1 = X1.C1 * (-delta1 + np.arctan2(self.vy + X1.a * self.r,
                                            self.vx - X1.d * self.r))
        fy2 = X1.C2 * (-delta2 + np.arctan2(self.vy + X1.a * self.r,
                                            self.vx + X1.d * self.r))
        fy3 = X1.C3 * (-delta3 + np.arctan2(self.vy - X1.b * self.r,
                                            self.vx - X1.d * self.r))
        fy4 = X1.C4 * (-delta4 + np.arctan2(self.vy - X1.b * self.r,
                                            self.vx + X1.d * self.r))
        fy = (fy1,fy2,fy3,fy4)
        
        f1cx = fx1 * np.cos(delta1) - fy1 * np.sin(delta1)
        f2cx = fx2 * np.cos(delta2) - fy2 * np.sin(delta2)
        f3cx = fx3 * np.cos(delta3) - fy3 * np.sin(delta3)
        f4cx = fx4 * np.cos(delta4) - fy4 * np.sin(delta4)
        f1cy = fx1 * np.sin(delta1) + fy1 * np.cos(delta1)
        f2cy = fx2 * np.sin(delta2) + fy2 * np.cos(delta2)
        f3cy = fx3 * np.sin(delta3) + fy3 * np.cos(delta3)
        f4cy = fx4 * np.sin(delta4) + fy4 * np.cos(delta4)
        sigma1 = f1cx + f2cx + f3cx + f4cx
        sigma2 = f1cy + f2cy + f3cy + f4cy

        # Calc derivatives
        psidot =  self.r
        edot   = -self.vy * np.cos(self.psi) - self.vx * np.sin(self.psi)
        ndot   =  self.vx * np.cos(self.psi) - self.vy * np.sin(self.psi)
        vxdot  =  self.r  * self.vy + 1 / X1.M * sigma1
        vydot  = -self.r  * self.vx + 1 / X1.M * sigma2
        rdot   =  1/X1.Iz * ( X1.d  * ( - f1cx + f2cx - f3cx + f4cx )
                            + X1.a  * (   f1cy + f2cy               )
			    - X1.b  * (   f3cy + f4cy               ) )

        # Calc next state
        self.psi_n   = self.psi   + self.dt * psidot 
        self.east_n  = self.east  + self.dt * edot 
        self.north_n = self.north + self.dt * ndot 
        self.r_n     = self.r     + self.dt * rdot 
        self.vx_n    = self.vx    + self.dt * vxdot 
        self.vy_n    = self.vy    + self.dt * vydot 

        """ IS THIS NECESSARY? - aLSTERDA"""
        # update inner value
        self.setStateValue( self.psi_n, self.east_n, self.north_n,
                            self.r_n,   self.vx_n,   self.vy_n     )

        # return next state
        return [ self.psi_n, self.east_n, self.north_n, 
                 self.r_n,   self.vx_n,   self.vy_n,    fy ]

    def setState(self, state):
        # set class structured state according to dictionary
        self.psi   = state['psi']
        self.east  = state['E'  ]
        self.north = state['N'  ]
        self.vx    = state['Ux' ]
        self.vy    = state['Uy' ]
        self.r     = state['r'  ]

    def getState(self):
        # return state as dictionary
        state = {'E'    : self.east ,		# [m]     pos East
                 'N'    : self.north,		# [m]     pos North
                 'psi'  : self.psi  ,		# [rad]   heading
                 'Ux'   : self.vx   ,		# [m/s]   longitudinal speed
                 'Uy'   : self.vy   ,		# [m/s]   lateral speed
                 'r'    : self.r     }          # [rad/s] yawrate
        return state
        
    def simulate_T(self, del_cmd, Fxr, T):
        # set command to simulator
        self.fx = [0,0,Fxr,Fxr]
        self.delta_4ws = del_cmd

        # simulate
	for i in range(int(T/self.dt)):
	    self.simulate_dt()
        
        # return state
        state = {'E'    : self.east_n ,		# [m]     pos East
                 'N'    : self.north_n,	        # [m]     pos North
                 'psi'  : self.psi_n  ,		# [rad]   heading
                 'Ux'   : self.vx_n   ,		# [m/s]   longitudinal speed
                 'Uy'   : self.vy_n   ,		# [m/s]   lateral speed
                 'r'    : self.r       }        # [rad/s] yawrate
        return state

class LinearSimulator:
    """
    Linearized 4 Wheel Vehicle Simulation Class
      x_dot = A*x + B*u + C
      x = [ e dPsi Uy r ] ; u = [ delta_fl, delta_fr, delta_rl, delta_rr ]
      see supportDocs/vehicleModels.pdf for derivation
    """
    def __init__(self):
        X1()
    
    def simulate_Lin(self, local_state_0, Ux, path, delta, dt):
        a  = X1.a
        b  = X1.b
        M  = X1.M
        Iz = X1.Iz
        Cf = X1.Cf
        Cr = X1.Cr
        Q  =  (Cr*b   - Cf*a  )/M /Ux 
        R  = -(Cf*a*a + Cr*b*b)/Iz/Ux 
        C  =  Cf + Cr
        I  =  np.identity(4) 
        A  = np.matrix([ [0, Ux,      1,    0], 
                         [0, 0 ,      0,    1], 
                         [0, 0 ,-C/M/Ux, Q-Ux], 
                         [0, 0 ,   Q/Iz,    R] ])                # dynamics 
        B  = np.matrix([ [      0,       0,       0,       0], 
                         [      0,       0,       0,       0], 
                         [   Cf/M,    Cf/M,    Cr/M,    Cr/M], 
                         [a*Cf/Iz, a*Cf/Iz,-b*Cr/Iz,-b*Cr/Iz] ]) # control 
        C  = np.matrix([ 0           ,
                         path['k']*Ux,
                         0           ,
                         0             ])                        # affine

        local_state_1 = ( I + dt*A ).dot(local_state_0) \
                      + dt * ( B.dot(delta) + C )

        return local_state_1, B

    def getLocalState(self, state, path):
	local_state = np.array([ctrl.calcLateralError(path,state),
                                ctrl.calcHeadingError(path,state),
                                state['Uy']                      ,
                                state['r']                        ])
        return local_state

