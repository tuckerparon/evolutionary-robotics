#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyze
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import numpy
import matplotlib.pyplot as plt

# load in sensor values .npy file
backLegSensorValues = numpy.load('data/backLegSensorValues.npy')
frontLegSensorValues = numpy.load('data/frontLegSensorValues.npy')

# print sensor values
print(backLegSensorValues)
print(frontLegSensorValues)

# plot sensor values
plt.plot(backLegSensorValues, label='Back Leg', linewidth=1)
plt.plot(frontLegSensorValues, label='Front Leg', linewidth=1)
plt.legend()
plt.show()
