#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

class Storage:
    def __init__(self):
        self.t = []
        self.E = []
        self.N = []
        self.Psi = []
        self.Ux = []
        self.dPsi = []
        self.e = []

    def store(self,state):
        self.t  = np.r_[self.t, state['t']]
        self.E  = np.r_[self.E, state['E']]
        self.N  = np.r_[self.N, state['N']]
        self.Psi = np.r_[self.Psi, state['psi']]
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

        if hasattr(self, 'Fx'):
            self.Fx = np.vstack([self.Fx, np.array(state['Fx'])])
        else:   
            self.Fx = np.array(state['Fx'])            

        if hasattr(self, 'Fy'):
            self.Fy = np.vstack([self.Fy, np.array(state['Fy'])])
        else:   
            self.Fy = np.array(state['Fy'])            

class Plotter:
    """ Experiment Plot Class """
    def __init__(self):
        pass

    def plot(self, storage):
        plt.subplot(611)
        plt.plot(storage.E,storage.N)
        plt.axis('equal')
        plt.ylabel('Course Shape')
        plt.grid()

        plt.subplot(612)
        plt.plot(storage.Ux)
        plt.ylabel('Speed [m/s]')
        plt.grid()

        plt.subplot(613)
        plt.plot(storage.dPsi/np.pi*180.0)
        plt.ylabel('Heading Error [deg]')
        plt.grid()

        plt.subplot(614)
        plt.plot(storage.e)
        plt.ylabel('Lateral Error [m]')
        plt.grid()

        plt.subplot(615)
        plt.plot(storage.delta[:,0]/np.pi*180.0,label='FrontLeft')
        plt.plot(storage.delta[:,1]/np.pi*180.0, label='FrontRight')
        plt.plot(storage.delta[:,2]/np.pi*180.0, label='RearLeft')
        plt.plot(storage.delta[:,3]/np.pi*180.0, label='RearRight')
        plt.ylabel('Steer Angle [deg]')
        plt.legend()
        plt.grid()

        plt.figure()
        #plt.subplot(616)
        plt.plot(storage.guess[:,0]/np.pi*180.0,label='GuessFL')
        plt.plot(storage.guess[:,1]/np.pi*180.0,label='GuessFR')
        plt.plot(storage.guess[:,2]/np.pi*180.0,label='GuessRL')
        plt.plot(storage.guess[:,3]/np.pi*180.0,label='GuessRR')
        plt.plot(storage.misalign[:,0]/np.pi*180.0,label='MisFL')
        plt.plot(storage.misalign[:,1]/np.pi*180.0,label='MisFR')
        plt.plot(storage.misalign[:,2]/np.pi*180.0,label='MisRL')
        plt.plot(storage.misalign[:,3]/np.pi*180.0,label='MisRR')
        plt.ylabel('Guess/ Align [deg]')
        plt.legend()
        plt.grid()
        
                
        #plt.tight_layout()
        plt.show()
	#plt.draw()        
