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
    
def Create_Robot():
    
    # create urdf file to store description of robots body
    pyrosim.Start_URDF("body.urdf")
    
    # declare variables
    length = 1
    width = 1
    height = 1
    
    # create link 0
    pyrosim.Send_Cube(name="Torso", pos=[0, 1, 1.5] , size = [length, width, height])
    
    # create link between 0 and 1
    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0, .5, 1])

    # create link 1
    pyrosim.Send_Cube(name="BackLeg", pos=[0, -.5, -.5] , size = [length, width, height])
        
    # create link between 1 and 2
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0, 1.5, 1])

    # create link 2
    pyrosim.Send_Cube(name="FrontLeg", pos=[0, .5, -.5] , size = [length, width, height])
    
    # end
    pyrosim.End()
    
    
Create_World()
Create_Robot()
