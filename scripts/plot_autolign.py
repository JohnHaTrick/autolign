#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

class Plotter:
    """ Experiment Plot Class """
    def __init__(self):
        self.t = []
        self.E = []
        self.N = []
        self.Ux = []
        self.dPsi = []
        self.e = []

    def store(self,state):
        self.t  = np.r_[self.t, state['t']]
        self.E  = np.r_[self.E, state['E']]
        self.N  = np.r_[self.N, state['N']]
        self.Ux = np.r_[self.Ux,state['Ux']]
        self.dPsi    = np.r_[self.dPsi, state['dPsi']]
        self.e  = np.r_[self.e, state['e']]

        if hasattr(self, 'delta'):
            self.delta = np.vstack([self.delta, np.array(state['delta'])])
        else:   
            self.delta = np.array(state['delta'])            

        if hasattr(self, 'guess'):
            self.guess = np.vstack([self.guess, np.array(state['guess'])])
        else:   
            self.guess = np.array(state['guess'])            

        if hasattr(self, 'misalign'):
            self.misalign = np.vstack([self.misalign, np.array(state['misalign'])])
        else:   
            self.misalign = np.array(state['misalign'])            




    def plot(self):
        plt.subplot(611)
        plt.plot(self.E,self.N)
        plt.axis('equal')
        plt.ylabel('Course Shape')
        plt.grid()

        plt.subplot(612)
        plt.plot(self.Ux)
        plt.ylabel('Speed [m/s]')
        plt.grid()

        plt.subplot(613)
        plt.plot(self.dPsi/np.pi*180.0)
        plt.ylabel('Heading Error [deg]')
        plt.grid()

        plt.subplot(614)
        plt.plot(self.e)
        plt.ylabel('Lateral Error [m]')
        plt.grid()

        plt.subplot(615)
        plt.plot(self.delta/np.pi*180.0)
        plt.ylabel('Steer Angle [deg]')
        plt.legend({'FrontLeft','FrontRight','RearLeft','RearRight'})
        plt.grid()

        plt.subplot(616)
        plt.plot(self.guess/np.pi*180.0)
        plt.plot(self.misalign/np.pi*180.0)
        plt.ylabel('Guess/ Align [deg]')
        plt.legend({'GuessFL','GuessFR','GuessRL','GuessRR',
                    'MisFL','MisFR','MisRL','MisRR',})
        plt.grid()
        
                
        #plt.tight_layout()
        plt.show()        