#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Solution
CS206: Evolutionary Robotics

@author: tuckerparon
"""

import numpy as np
import pyrosim.pyrosim as pyrosim
import random
import os
import constants as c
import time

class SOLUTION:
    def __init__(self, nextAvailableID):
        self.weights = np.random.rand(3, 2)
        self.weights = self.weights * 2 - 1
        self.myID = nextAvailableID

    def Evaluate(self, directOrGUI):
        pass

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        os.system("python3 simulate.py " + directOrGUI + " " + str(self.myID) + " &")

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.01)
        fitnessFile = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()
        os.system("rm fitness" + str(self.myID) + ".txt")

    def Create_World(self):

        # store file
        pyrosim.Start_SDF("world.sdf")

        # create box
        pyrosim.Send_Cube(name="Box", pos=[c.x, c.y, c.z], size = [c.length, c.width, c.height])

        # close file
        pyrosim.End()

    def Generate_Body(self):


        # create urdf file to store description of robots body
        pyrosim.Start_URDF("body.urdf")

        # create link 0
        pyrosim.Send_Cube(name="Torso", pos=[0, 1, 1.5] , size = [c.length, c.width, c.height])

        # create link between 0 and 1
        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [-.5, 1, 1])

        # create link 1
        pyrosim.Send_Cube(name="BackLeg", pos=[-.5, 0, -.5] , size = [c.length, c.width, c.height])

        # create link between 1 and 2
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [.5, 1, 1])

        # create link 2
        pyrosim.Send_Cube(name="FrontLeg", pos=[.5, 0, -.5] , size = [c.length, c.width, c.height])

        # end
        pyrosim.End()

    def Generate_Brain(self):

        # create nndf file to store brain contents
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName="FrontLeg")

        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

        #pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 3 , weight = -1.0 )
        #pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 3 , weight = -1.0)
        #pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 4 , weight = -1.0)
        #pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 4 , weight = -1.0)

        # loop over sensor neurons
        for currentRow in range(self.weights.shape[0]):
            # loop over the motors
            for currentColumn in range(self.weights.shape[1]):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn + 3, weight = self.weights[currentRow][currentColumn])

        # end
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0, 2)
        randomColumn = random.randint(0, 1)
        self.weights[randomRow,randomColumn] = random.random() * 2 -1

    def Set_ID(self):
        return self.myID