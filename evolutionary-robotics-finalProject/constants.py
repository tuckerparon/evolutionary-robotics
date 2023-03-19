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
x = 0
y = 0
z = 1

# simulation time steps
time_steps = 1000

# target angles
targetAngles = 2 * (numpy.sin(numpy.linspace(0, 2*numpy.pi, time_steps)) * numpy.pi / 4) # scale them by pi/4

# number of generation
numberOfGenerations = 6

# population size
populationSize = 5

# sleep rate
sleepRate = 1/60

numSensorNeurons = 9
numMotorNeurons = 8

motorJointRange = .5
