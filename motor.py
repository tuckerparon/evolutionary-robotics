#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor
CS 206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import numpy
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p

# motor.py
class MOTOR:
    def __init__(self, jointName):

        # set joint name and get ready to act
        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):

        # initialize motor value array
        self.motorValues = numpy.zeros(c.time_steps)

        # retrieve motor constants for calculation
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.offset = c.phaseOffset
        self.targetAngles = c.targetAngles

        # line 105 in refactoring instructions - make one motor twice as fast
        if self.jointName == b'Torso_BackLeg':
            self.frequency *= 2

        # calculate motor values
        for i in range(c.time_steps):
            self.motorValues[i] = self.amplitude * numpy.sin(self.frequency * self.targetAngles[i] + self.offset)


    def Set_Value(self, robot, t):

        # set motor values for appropriate joint
        pyrosim.Set_Motor_For_Joint(bodyIndex = robot, jointName = self.jointName, controlMode = p.POSITION_CONTROL, targetPosition = self.motorValues[t], maxForce = 50)

    def Save_Values(self):

        # save values
        numpy.save('data//motorValues.npy', self.motorValues)