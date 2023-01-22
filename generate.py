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
pyrosim.Start_SDF("box.sdf")

# create box
pyrosim.Send_Cube(name="Box", pos=[0,0,0.5] , size=[1,1,1])

# close file
pyrosim.End()
