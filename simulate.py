#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulate
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import pybullet as p
import time
import pybullet_data

# create physics client
physicsClient = p.connect(p.GUI)

# get data path to access .urdf file for 'floor'
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# establish gravity
p.setGravity(0,0,-9.8)

# add a 'floor' to the world for the box to fall onto
planeId = p.loadURDF("plane.urdf")

# read in described world
p.loadSDF("boxes.sdf")

# update simulation across 1000 time steps
for i in range(1000):
    
    print(i)
    time.sleep(1/60)
    p.stepSimulation()
    

# disconnect
p.disconnect()
