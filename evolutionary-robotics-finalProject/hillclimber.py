#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hill Climber
CS206: Evolutionary Robotics

@author: tuckerparon
"""

from solution import SOLUTION
import constants as c
import copy
import random

class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()
        self.child = None

    def Evolve(self):
        self.parent.Evaluate('GUI')
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()
        self.Show_Best()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate('DIRECT')
        print('About to Print.')
        self.Print()
        print('Just Printed.')
        self.Select()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child

    def Print(self):
        print('PRINTING:')
        print('Parent:', self.parent.fitness, '<', 'Child:', self.child.fitness)

    def Show_Best(self):
        self.parent.Evaluate('GUI')

