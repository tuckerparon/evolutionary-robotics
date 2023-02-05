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
amplitude = .4
frequency = 20
phaseOffset = 0

# simulation time steps
time_steps = 1000

# target angles
targetAngles = (numpy.sin(numpy.linspace(0, 2*numpy.pi, time_steps)) * numpy.pi / 4) # scale them by pi/4