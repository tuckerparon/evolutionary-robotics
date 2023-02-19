#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Search
CS206: Evolutionary Robotics

@author: tuckerparon
"""

from hillclimber import HILL_CLIMBER
from parallelHillClimber import PARALLEL_HILL_CLIMBER

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Show_Best()
