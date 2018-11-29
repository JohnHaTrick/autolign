#!/usr/bin/env python

import numpy as np
import ctrl_autolign as ctrl

# sim_autolign.py provides modelling parameters and models for X1

def ackermann(delta_f,delta_r,d,l):
    # Linear model of ackermann angle
    # Small angle approx : delta_f,r,fl,fr,rl,rr << 1
    #    allows tan, tan-1 ==> 1. 
    # Input
    #   d: half length of tread
    #   l: wheel base
    x        = d / l * ((delta_f) - (delta_r))
    delta_fl = delta_f / (1 - x)
    delta_fr = delta_f / (1 + x) 
    delta_rl = delta_r / (1 - x) 
    delta_rr = delta_r / (1 + x)
    return delta_fl, delta_fr, delta_rl, delta_rr 

class X1Simulator:
    """ Vehicle Simulation Class """

    def __init__(self):
        ## Vehicle parameter    
        self.M = 1964.0		    # Mass         [kg]
        self.a = 1.50		    # Front length [m]
        self.b = 1.37		    # Rear length  [m]
        self.l = self.a+self.b      # Wheel base   [m]
        self.d = 0.82		    # Half Tread   [m]
        self.Iz = 2900.0	    # Inertia      [kg-m^2] 2250 = 1000*1.5*1.5
        self.Cf = -140000.0	    # Cornering Stiffness [N/rad]
        self.Cr = -190000.0	    # Cornering Stiffness [N/rad]
        self.C1 = self.Cf/2.0	    # Cornering Stiffness of Front Left  [N/rad]
        self.C2 = self.Cf/2.0	    # Cornering Stiffness of Front Right [N/rad]
        self.C3 = self.Cr/2.0	    # Cornering Stiffness of Rear Left   [N/rad]
        self.C4 = self.Cr/2.0	    # Cornering Stiffness of Rear Right  [N/rad]
	self.C  = self.Cf + self.Cr


        ## Simulation Parameter 
        self.dt = 0.001		    # Time step [s]

	# Linear model: x_dot = A*x + B*u + C
	#   x = [ e dPsi Uy r ] ; u = [ delta_fl, delta_fr, delta_rl, delta_rr ]
	#   see supportDocs/vehicleModels.pdf for derivation
	
	self.A = np.matrix([
                            [0, Ux,      1,    0],
                            [0, 0 ,      0,    1],
                            [0, 0 ,-C/M/Ux, Q-Ux],
                            [0, 0 ,   Q/Iz,    R]
                           ])                        # dynamics
	self.B = np.matrix([
                            [      0,       0,       0,       0],
                            [      0,       0,       0,       0],
                            [   Cf/M,    Cf/M,    Cr/M,    Cr/M],
                            [a*Cf/Iz, a*Cf/Iz,-b*Cr/Iz,-b*Cr/Iz]
                           ])                        # control
     C = np.matrix([
                     0,
                     path['k']*Ux,
                     0,
                     0
                   ])                        # affine

        self.east     = 0.0
        self.north    = 0.0
        self.psi      = 0.0
        self.vx       = 0.0
        self.vy       = 0.0
        self.yawrate  = 0.0

        self.fx       = [0.0, 0.0, 0.0, 0.0]
        self.delta_fr = [0.0, 0.0]
    
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

    def simulate(self):
        # Calc ackermann steer angle of 4 wheels, in case of bicycle commands
        # self.delta_4ws = ackermann(self.delta_fr[0], self.delta_fr[1], self.d, self.l)
        [delta1, delta2, delta3, delta4] = self.delta_4ws
        [fx1, fx2, fx3, fx4]             = self.fx
        
        # Calc forces to each tire    
        fy1 = self.C1 * (-delta1 + np.arctan2(self.vy + self.a * self.r, self.vx - self.d * self.r))
        fy2 = self.C2 * (-delta2 + np.arctan2(self.vy + self.a * self.r, self.vx + self.d * self.r))
        fy3 = self.C3 * (-delta3 + np.arctan2(self.vy - self.b * self.r, self.vx - self.d * self.r))
        fy4 = self.C4 * (-delta4 + np.arctan2(self.vy - self.b * self.r, self.vx + self.d * self.r))

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
        psidot = self.r
        edot   = -self.vy * np.cos(self.psi) - self.vx * np.sin(self.psi)
        ndot   =  self.vx * np.cos(self.psi) - self.vy * np.sin(self.psi)
        rdot   = 1 / self.Iz * ( self.d * ( - f1cx + f2cx - f3cx + f4cx )
                               + self.a * (   f1cy + f2cy               )
			       - self.b * (   f3cy + f4cy               ) )
        vxdot  =  self.r * self.vy + 1 / self.M * sigma1
        vydot  = -self.r * self.vx + 1 / self.M * sigma2

        # Calc next state
        self.psi_n   = self.psi   + self.dt * psidot 
        self.east_n  = self.east  + self.dt * edot 
        self.north_n = self.north + self.dt * ndot 
        self.r_n     = self.r     + self.dt * rdot 
        self.vx_n    = self.vx    + self.dt * vxdot 
        self.vy_n    = self.vy    + self.dt * vydot 

        # update inner value
        self.setStateValue(self.psi_n,self.east_n,self.north_n,self.r_n,self.vx_n,self.vy_n)

        # return next state
        return [self.psi_n, self.east_n, self.north_n, self.r_n, self.vx_n, self.vy_n, fy]

    def setState(self, state):
        self.psi   = state['psi']
        self.east  = state['E']
        self.north = state['N']
        self.vx    = state['Ux']
        self.vy    = state['Uy']
        self.r     = state['r']

    def getState(self):
        # return state
        state= {'E'    :  self.east,			# [m]   pos East. Durand Datum
                'N'    :  self.north,			# [m]   pos North
                'psi'  :  self.psi,			# [rad] heading
                'Ux'   :  self.vx,			# [m/s] longitudinal speed
                'Uy'   :  self.vy,			# [m/s] lateral speed
                'r'    :  self.r,           # [rad/s] yawrate
           }
        return state
        
    def getLocalState(self, state, path):
	local_state = np.array([ctrl.calcLateralError(path,memory.state_0),
                                ctrl.calcHeadingError(path,memory.state_0),
                                memory.state_0['Uy']                      ,
                                memory.state_0['r']                       ])

    def simulateX1(self, del_cmd, Fxr, dt):
        # set command to simulator
        self.fx = [0,0,Fxr,Fxr]
        self.delta_4ws = del_cmd

        # simulate
	for i in range(int(dt/self.dt)):
	    self.simulate()
        
        # return state
        state= {'E'    :  self.east_n,			# [m]   pos East. Durand Datum
                'N'    :  self.north_n,			# [m]   pos North
                'psi'  :  self.psi_n,			# [rad] heading
                'Ux'   :  self.vx_n,			# [m/s] longitudinal speed
                'Uy'   :  self.vy_n,			# [m/s] lateral speed
                'r'    :  self.r,               # [rad/s] yawrate
           }
        return state
