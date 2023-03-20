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
import os
import sys

from solution import SOLUTION
import constants as c
import copy
import os
import sys

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf") # remove temporary brain files
        os.system("rm fitness*.nndf") # remove temporary fitness files

        self.parents = {}
        self.nextAvailableID = 0
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
        #self.child = None

    def Evolve(self):
        #self.parent.Evaluate('GUI')
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()
        #self.Show_Best()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        #print('About to Print.')
        self.Print()
        #print('Just Printed.')
        self.Select()

    def Spawn(self):
        self.children = {}
        for i in range(len(self.parents)):
            self.parents[i].Set_ID()
            self.children[i] = copy.deepcopy(self.parents[i])
            self.nextAvailableID += 1

    def Mutate(self):
        for i in range(len(self.children)):
            self.children[i].Mutate()

    def Select(self):
        for key in range(len(self.parents)):
            if self.parents[key].fitness > self.children[key].fitness:
                self.parents[key] = self.children[key]

    def Print(self):
        print('\n')
        for key in range(len(self.parents)):
            print('Parent:', self.parents[key].fitness, '<', 'Child:', self.children[key].fitness)
        print('\n')

    def Show_Best(self):
        bestkey = 0
        bestFit = self.parents[0].fitness
        for key in range(len(self.parents)):
            if self.parents[key].fitness < bestFit:
                bestFit = self.parents[key].fitness
                bestkey = key
        self.parents[bestkey].Start_Simulation("GUI")

    def Evaluate(self, solutions):
        for i in range(len(solutions)):
            solutions[i].Start_Simulation('DIRECT')
        for i in range(len(solutions)):
            solutions[i].Wait_For_Simulation_To_End()

#%%
