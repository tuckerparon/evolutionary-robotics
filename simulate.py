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

# create physics client
physicsClient = p.connect(p.GUI)

# read in described world
p.loadSDF("box.sdf")

# update simulation across 1000 time steps
for i in range(1000):
    
    print(i)
    time.sleep(1/60)
    p.stepSimulation()
    

# disconnect
p.disconnect()
