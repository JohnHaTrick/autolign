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
        plt.plot(self.delta[:,0]/np.pi*180.0,label='FrontLeft')
        plt.plot(self.delta[:,1]/np.pi*180.0, label='FrontRight')
        plt.plot(self.delta[:,2]/np.pi*180.0, label='RearLeft')
        plt.plot(self.delta[:,3]/np.pi*180.0, label='RearRight')
        plt.ylabel('Steer Angle [deg]')
        plt.legend()
        plt.grid()

        plt.figure()
        #plt.subplot(616)
        plt.plot(self.guess[:,0]/np.pi*180.0,label='GuessFL',color='blue')
        plt.plot(self.guess[:,1]/np.pi*180.0,label='GuessFR',color='purple')
        plt.plot(self.guess[:,2]/np.pi*180.0,label='GuessRL',color='red')
        plt.plot(self.guess[:,3]/np.pi*180.0,label='GuessRR',color='green')
        plt.plot(self.misalign[:,0]/np.pi*180.0,label='MisFL',color='blue',  linestyle='dashed')
        plt.plot(self.misalign[:,1]/np.pi*180.0,label='MisFR',color='purple',linestyle='dashed')
        plt.plot(self.misalign[:,2]/np.pi*180.0,label='MisRL',color='red',   linestyle='dashed')
        plt.plot(self.misalign[:,3]/np.pi*180.0,label='MisRR',color='green', linestyle='dashed')
        plt.ylabel('Guess/ Align [deg]')
        plt.legend()
        plt.grid()
                
        #plt.tight_layout()
        plt.show()        
