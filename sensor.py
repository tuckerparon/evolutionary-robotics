#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sensor
CS 206: Evolutionary Robotics


@author: tuckerparon
"""

# imports
import numpy
import constants as c
import pyrosim.pyrosim as pyrosim

# sensor.py
class SENSOR:
    def __init__(self, linkName):

        # intialize linkName and link values
        self.linkName = linkName
        self.values = numpy.zeros(c.time_steps)

    def Get_Value(self, t):

        # get values at each time step and print final vectors
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName) # sensor values
        if t == c.time_steps-1:
            print(self.values)

    def Save_Values(self):

        # save values
        numpy.save('data//sensorValues.npy', self.values)