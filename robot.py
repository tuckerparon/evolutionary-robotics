#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot
CS 206: Evolutionary Robotics


@author: tuckerparon
"""

from sensor import SENSOR
from motor import MOTOR
import pyrosim.pyrosim as pyrosim
import pybullet as p

# robot.py
class ROBOT:
    def __init__(self):

        # for sensor and motor values
        self.sensors = {}
        self.motors = {}

        # set up robot
        self.robot = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robot)

        # get ready to sense and act
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self):

        # fill in sensor values (as value) for each link (as key)
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):

        # get sensor value for each link at each time step
        for i, s in enumerate(self.sensors.values()):
            s.Get_Value(t)

    def Prepare_To_Act(self):

        # fill in motor values (as value) for each joint (as key)
        for jointName in pyrosim.jointNamesToIndices:
            print(jointName)
            self.motors[jointName] = MOTOR(jointName)

    def Act(self, t):

        # get motor value for each joint at each time step
        for i, m in enumerate(self.motors.values()):
            m.Set_Value(self.robot, t)

    def Save_Values(self):

        # save the values
        for key in self.sensors:
            self.sensors[key].Save_Values()

        for key in self.motors:
            self.motors[key].Save_Values()


