#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OneLink
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import pyrosim.pyrosim as pyrosim

# store file
pyrosim.Start_SDF("boxes.sdf")

# declare variables
length = 1
width = 1
height = 1

x = 0
y = 0
z = .50

# create box
for i in range(10):
    name = 'Box' + str(i)

    length = (.9**i) * length
    width = (.9**i) * width
    height = (.9**i) * height
    
    for i2 in range(5):
        for i3 in range(5):
            pyrosim.Send_Cube(name=name, pos=[i2, i3, z] , size = [length, width, height])
            print(x,y,z,i3)

    z = z + height * .95

# close file
pyrosim.End()
