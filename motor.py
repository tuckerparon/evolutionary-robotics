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

    def Set_Value(self, robot, desiredAngle):

        # set motor values for appropriate joint
        pyrosim.Set_Motor_For_Joint(bodyIndex = robot, jointName = self.jointName, controlMode = p.POSITION_CONTROL, targetPosition = desiredAngle, maxForce = 50)
