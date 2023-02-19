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

print(sys.argv)

# store GUI/DIRECT variable
directOrGUI = sys.argv[1]

# store ID
solutionID = sys.argv[2]

# start simulation
simulation = SIMULATION(directOrGUI, solutionID)
simulation.run()
simulation.Get_Fitness()
