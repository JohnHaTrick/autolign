#!/usr/bin/env python

# This script should act like the communications portion of the autolign codes
import numpy as np

def ackermann(delta_f,delta_r,d,l):
    # Linear model of ackermann angle
    # Not including (inverse)trigonometric functions
    # Assume delta_f,r,fl,fr,rl,rr << 1
    # then tan, tan-1 ==> 1.
    # Input
    # d: half length of tread
    # l: wheel base
    x = d / l * ((delta_f) - (delta_r));
    delta_fl = delta_f / (1 - x);
    delta_fr = delta_f / (1 + x) 
    delta_rl = delta_r / (1 - x) 
    delta_rr = delta_r / (1 + x)
    return delta_fl, delta_fr, delta_rl, delta_rr 


class X1Simulator:
    """ Vehicle Simulation Class """

    def __init__(self):
        ## Vehicle parameter    
        self.M = 1964.0   # Mass         [kg]
        self.a = 1.50     # Front length [m]
        self.b = 1.37     # Rear length  [m]
        self.l = self.a+self.b      # Wheel base   [m]
        self.d = 0.82     # Half Tread   [m]
        self.Iz = 2900.0    # Inertia      [kg-m^2] 2250 = 1000*1.5*1.5
        self.Cf = -140000.0 # Cornering Stiffness [N/rad]
        self.Cr = -190000.0 # Cornering Stiffness [N/rad]
        self.C1 = self.Cf/2.0  # Cornering Stiffness of Front Left  [N/rad]
        self.C2 = self.Cf/2.0  # Cornering Stiffness of Front Right [N/rad]
        self.C3 = self.Cr/2.0  # Cornering Stiffness of Rear Left   [N/rad]
        self.C4 = self.Cr/2.0  # Cornering Stiffness of Rear Right  [N/rad]

        ## Simulation Parameter 
        self.dt = 0.001            # Time step [s]

        self.east = 0.0
        self.north = 0.0
        self.psi = 0.0
        self.vx = 15.0
        self.vy = 0.0
        self.yawrate = 0.0

        self.fx = [0.0, 0.0, 0.0, 0.0]
        self.delta_fr = [0.0, 0.0]
    
    def setStateValue(self,psi,east,north,r,vx,vy):
        self.psi = psi
        self.east = east
        self.north = north
        self.r = r
        self.vx = vx
        self.vy = vy
            
    def setCommand(self,fx,delta_fr):
        # fx:       List of 4 longitudinal forces to each wheel(FL,FR,RL,RR)
        # delta_fr: List of 2 steering angles to each axles(Front, Rear)
        self.fx = fx
        self.delta_fr = delta_fr

    def simulate(self):
        # Calc ackermann steer angle of 4 wheels
        # self.delta_4ws = ackermann(self.delta_fr[0], self.delta_fr[1], self.d, self.l) # in case of front and rear
        [delta1, delta2, delta3, delta4] = self.delta_4ws
        [fx1, fx2, fx3, fx4] = self.fx
        
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

        # Calc transition
        psidot = self.r
        edot = -self.vy * np.cos(self.psi) - self.vx * np.sin(self.psi)
        ndot =  self.vx * np.cos(self.psi) - self.vy * np.sin(self.psi)
        rdot = 1 / self.Iz * ( self.d * ( - f1cx + f2cx - f3cx + f4cx ) \
                               + self.a * ( f1cy + f2cy )- self.b * ( f3cy + f4cy) )
        vxdot =  self.r * self.vy + 1 / self.M * sigma1
        vydot = -self.r * self.vx + 1 / self.M * sigma2

        # Calc next state
        self.psi_n = self.psi + psidot * self.dt
        self.east_n = self.east + edot * self.dt
        self.north_n = self.north + ndot * self.dt
        self.r_n = self.r + rdot * self.dt
        self.vx_n = self.vx + vxdot * self.dt
        self.vy_n = self.vy + vydot * self.dt

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
        state= {'E'    :  self.east,			# [m]   init pos East. Durand Datum
                'N'    :  self.north,			# [m]   init pos North
                'psi'  :  self.psi,			# [rad] init heading
                'Ux'   :  self.vx,			# [1/m] curvature
                'Uy'   :  self.vy,			# [m/s] desired speed
           }
        return state
        
    def simulateX1(self, del_cmd, Fxr):
        # set command to simulator
        self.fx = [0,0,Fxr,Fxr]
        self.delta_4ws = del_cmd

        # simulate
        self.simulate()
        
        # return state
        state= {'E'    :  self.east_n,			# [m]   init pos East. Durand Datum
                'N'    :  self.north_n,			# [m]   init pos North
                'psi'  :  self.psi_n,			# [rad] init heading
                'Ux'   :  self.vx_n,			# [1/m] curvature
                'Uy'   :  self.vy_n,			# [m/s] desired speed
           }
        return state