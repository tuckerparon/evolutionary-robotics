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

# load in angle/control vectors
targetAngles =  numpy.load('data/targetAngles.npy')
backLegMotorValues =  numpy.load('data/backLegMotorValues.npy')
frontLegMotorValues =  numpy.load('data/frontLegMotorValues.npy')

# print sensor values
#print(backLegSensorValues)
#print(frontLegSensorValues)

# print angle/control values
#print(targetAngles)
#print(backLegMotorValues)
#print(frontLegMotorValues)

# plot sensor values
#plt.plot(backLegSensorValues, label='Back Leg', linewidth=1)
#plt.plot(frontLegSensorValues, label='Front Leg', linewidth=1)
#plt.plot(targetAngles, label='Angles', linewidth=1)
plt.plot(backLegMotorValues, label='Back Leg', linewidth=1)
plt.plot(frontLegMotorValues, label='Front Leg', linewidth=1)
plt.legend()
plt.show()
