#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Constants
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import numpy

# front leg angle/motor variables
amplitude = numpy.pi / 4
frequency = 20
phaseOffset = 0

# create world variables
length = 1
width = 1
height = 1
x = -5
y = -5
z = .50

# simulation time steps
time_steps = 1000

# target angles
targetAngles = (numpy.sin(numpy.linspace(0, 2*numpy.pi, time_steps)) * numpy.pi / 4) # scale them by pi/4

# number of generation
numberOfGenerations = 10
