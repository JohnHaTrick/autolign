#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

class Plotter:
    """ Experiment Plot Class """
    def __init__(self):
        self.E = []
        self.N = []
        self.UX = []

    def store(self,state):
        self.E = np.r_[self.E, state['E']]
        self.N = np.r_[self.N, state['N']]
        self.UX = np.r_[self.UX,state['Ux']]

    def plot(self):
        plt.subplot(211)
        plt.plot(self.E,self.N)
        plt.axis('equal')

        plt.subplot(212)
        plt.plot(self.UX)
        
        plt.tight_layout()
        plt.show()