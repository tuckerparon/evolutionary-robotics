#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
World
CS 206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import pybullet as p

class WORLD:
    def __init__(self):

        # add a 'floor' to the world for the box to fall onto
        self.planeId = p.loadURDF("plane.urdf")

        # read in described world
        p.loadSDF("world.sdf")