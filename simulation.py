#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulation
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import pybullet as p
import time
import pybullet_data
import constants as c
from world import WORLD
from robot import ROBOT

class SIMULATION:
    def __init__(self, directOrGUI, solutionID, gait):

        self.directOrGUI = directOrGUI
        self.solutionID = solutionID
        self.gait = gait

        # check run type
        if directOrGUI == 'DIRECT':
            p.connect(p.DIRECT)
        else:
            p.connect(p.GUI)

        # get data path to access .urdf file for 'floor'
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # establish gravity
        p.setGravity(0,0,-9.8)

        # establish world and robot
        self.world = WORLD()
        self.robot = ROBOT(self.solutionID)

    def run(self):

        # step through simulation by sensing and acting
        for t in range(c.time_steps):
            #print(t)
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Think()
            self.robot.Act()
            if self.directOrGUI == 'GUI':
                time.sleep(c.sleepRate)

    def Get_Fitness(self):
        self.robot.Get_Fitness(self.gait)

    def __del__(self):

        # end simulation and save values
        p.disconnect()
