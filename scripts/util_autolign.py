#!/usr/bin/env python

# Useful random utility functions
import datetime
import time

def welcome(args):
     print ("\nStarting %s" % args[0][-11:-3])       # Welcome message
     try:                                            # Usr enter an arg?
         mode = args[1]                              # choose sim or exp mode
         if mode == 'sim' or mode == 'exp':
             print ("in %s mode\n" % args[1])
         else:
             print ("in sim mode (by default)\n")
             mode = 'sim'
     except:
         print ("in sim mode (by default)\n")
         mode = 'sim'
     return mode

def getfilename():
    return datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d-%H-%M-%S')

