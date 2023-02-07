#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import pyrosim.pyrosim as pyrosim

def Create_World():
    
    # store file
    pyrosim.Start_SDF("world.sdf")

    # declare variables
    length = 1
    width = 1
    height = 1
    
    x = -5
    y = -5
    z = .50

    # create box
    pyrosim.Send_Cube(name="Box", pos=[x, y, z] , size = [length, width, height])
    
    # close file
    pyrosim.End()
    
def Generate_Body():


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

def Generate_Brain():

    # create nndf file to store brain contents
    pyrosim.Start_NeuralNetwork("brain.nndf")
    print('yes')

    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1, linkName="BackLeg")
    pyrosim.Send_Sensor_Neuron(name = 2, linkName="FrontLeg")
    pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
    # end
    pyrosim.End()
    
    
Create_World()
Generate_Body()
Generate_Brain()
