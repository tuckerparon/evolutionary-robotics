#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulate
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
from simulation import SIMULATION
import sys

# store GUI/DIRECT variable
directOrGUI = sys.argv[1]

# start simulation
simulation = SIMULATION(directOrGUI)
simulation.run()
simulation.Get_Fitness()
