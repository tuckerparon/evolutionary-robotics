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
import pyrosim.pyrosim as pyrosim
import numpy
import sys

# create physics client
physicsClient = p.connect(p.GUI)

# get data path to access .urdf file for 'floor'
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# establish gravity
p.setGravity(0,0,-9.8)

# add a 'floor' to the world for the box to fall onto
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

# read in described world
p.loadSDF("world.sdf")

# prepare to simulate
pyrosim.Prepare_To_Simulate(robotId)

# create zero vectors to populate with sensor information
backLegSensorValues = numpy.zeros(1000)
frontLegSensorValues = numpy.zeros(1000)

# update simulation across 1000 time steps
for i in range(1000):
    
    print(i)
    time.sleep(1/60)
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg") # back leg sensor
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg") # back leg sensor

# disconnect
p.disconnect()

# print 
print(backLegSensorValues)
print(frontLegSensorValues)

# save sensor arrays
numpy.save("data/backLegSensorValues.npy", backLegSensorValues)
numpy.save("data/frontLegSensorValues.npy", frontLegSensorValues)
