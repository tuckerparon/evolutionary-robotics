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
import random

# establish variables
amplitude_front = .4
frequency_front = 20
phaseOffset_front = 0

amplitude_back = .4
frequency_back = 20
phaseOffset_back = .5

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

# target angles vector (original)
targetAngles = numpy.sin(numpy.linspace(0, 2*numpy.pi, 1000))
targetAngles = (targetAngles * numpy.pi / 4) # scale them by pi/4

# target angles vector (new)
frontLegMotorValues = numpy.zeros(1000)
backLegMotorValues = numpy.zeros(1000)

# populate new vector
for i in range(len(targetAngles)):
    frontLegMotorValues[i] = amplitude_front * numpy.sin((frequency_front * targetAngles[i]) + phaseOffset_front)
    backLegMotorValues[i] = amplitude_back * numpy.sin((frequency_back * targetAngles[i]) + phaseOffset_back)

# save angle/motor vectors
#numpy.save("data/targetAngles.npy", targetAngles)
#numpy.save("data/frontLegMotorValues.npy", frontLegMotorValues)
#numpy.save("data/backLegMotorValues.npy", backLegMotorValues)
    
# update simulation across 1000 time steps
for i in range(1000):
    
    print(i)
    time.sleep(1/60)
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg") # back leg sensor
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg") # back leg sensor
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = frontLegMotorValues[i], maxForce = 50)
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = backLegMotorValues[i], maxForce = 50)
    
# disconnect
p.disconnect()

# print sensor values
#print(backLegSensorValues)
#print(frontLegSensorValues)

# save sensor arrays
#numpy.save("data/backLegSensorValues.npy", backLegSensorValues)
#numpy.save("data/frontLegSensorValues.npy", frontLegSensorValues)
