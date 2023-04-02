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
import constants as c
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os

# robot.py
class ROBOT:
    def __init__(self, solutionID):

        # set solution ID
        self.solutionID = solutionID

        # for sensor and motor values
        self.sensors = {}
        self.motors = {}
        self.values = {}

        # set up robot
        self.robot = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robot)

        # get ready to sense and act
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

        # set up brain
        self.nn = NEURAL_NETWORK("brain" + solutionID + ".nndf")

        os.system("rm brain" + solutionID + ".nndf")

    def Prepare_To_Sense(self):

        # fill in sensor values (as value) for each link (as key)
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):

        # get sensor value for each link at each time step
        for key in self.sensors:
            self.values[t] = self.sensors[key].Get_Value(t)

    def Prepare_To_Act(self):

        # fill in motor values (as value) for each joint (as key)
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self):

        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName).encode('ASCII')
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(self.robot, desiredAngle)

    def Save_Values(self):

        # save the values
        for key in self.sensors:
            self.sensors[key].Save_Values()

        for key in self.motors:
            self.motors[key].Save_Values()


    def Think(self):
        self.nn.Update()
        #self.nn.Print()


    def Get_Fitness(self, gait):
        stateOfLinkZero = p.getLinkState(self.robot,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        yCoordinateOfLinkZero = positionOfLinkZero[2]

        # Calculate the stability of the robot
        totalMass = 0
        totalLinkDist = 0
        for linkIndex in range(1, p.getNumJoints(self.robot)):
            linkState = p.getLinkState(self.robot, linkIndex) # position, orientation, velo., etc.
            linkPosition = linkState[0] # position
            linkMass = p.getDynamicsInfo(self.robot, linkIndex)[0] # mass
            linkDist = abs(linkPosition[0] - positionOfLinkZero[0]) # distance between link and torso
            totalLinkDist += linkDist * linkMass
            totalMass += linkMass
        stability = totalLinkDist / totalMass # average distance between each link and torso (small val. = more stable)

        # Determine the fitness based on the gait
        if gait == 'trotting':
            fitness = xCoordinateOfLinkZero - 0.5 * abs(yCoordinateOfLinkZero) - 0.1 * stability
        elif gait == 'pronking':
            fitness = -yCoordinateOfLinkZero
        elif gait == 'cantering':
            fitness = xCoordinateOfLinkZero

        f = open("tmp" + str(self.solutionID) + ".txt", "w")
        f.write(str(fitness))
        f.close()
        os.rename("tmp"+str(self.solutionID)+".txt", "fitness"+str(self.solutionID)+".txt")




