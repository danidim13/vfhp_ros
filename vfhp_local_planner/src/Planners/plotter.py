#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

class Plotter(object):
    def __init__(self, vfhp=None):
        self.vfhp = vfhp

    def plot_active_grid(self, i):
        plt.figure(i)
        plt.pcolor(self.vfhp._active_grid().T, alpha=0.75, edgecolors='k',vmin=0,vmax=self.vfhp.const.C_MAX)
        plt.xlabel("X")
        plt.ylabel("Y", rotation='horizontal')
        plt.title("Active Window (occupancy)")

    def plot_active_window(self, i):
        plt.figure(i)
        plt.pcolor(self.vfhp.active_window[:,:,MAG].T, alpha=0.75, edgecolors='k')
        plt.xlabel("X")
        plt.ylabel("Y", rotation='horizontal')
        plt.title("Active Window (magnitude)")


    def plot_grid(self, i):
        plt.figure(i)
        plt.pcolor(self.vfhp.obstacle_grid.T, alpha=0.75, edgecolors='k',vmin=0,vmax=self.vfhp.const.C_MAX)
        plt.xlabel("X")
        plt.ylabel("Y", rotation='horizontal')
        plt.title("cuadrícula d")

    def plot_hist(self, i):
        plt.figure(i)
        phi = [self.vfhp.const.ALPHA*x for x in xrange(self.vfhp.const.HIST_SIZE)]
        low = [self.vfhp.const.T_LO for x in xrange(self.vfhp.const.HIST_SIZE)]
        high = [self.vfhp.const.T_HI for x in xrange(self.vfhp.const.HIST_SIZE)]
        #i = [a for a in range(len(self.vfhp.polar_hist))]
        #plt.polar(phi, self.vfhp.polar_hist, color='r')
        #plt.polar(phi, low, color='b')
        #plt.polar(phi, high, color='g')
        plt.polar(phi, self.vfhp.polar_hist, color='y')
        plt.polar(phi, low, color ='g')
        plt.polar(phi, high, color ='r')
        plt.title("Primary Polar Histogram")

    def plot_show(self):
        plt.show()
