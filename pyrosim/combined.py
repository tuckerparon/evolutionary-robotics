#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Pyrosim
CS 206: Evolutionary Robotics

@author: tuckerparon
'''

import pybullet as p

from pyrosim.nndf import NNDF

from pyrosim.linksdf  import LINK_SDF

from pyrosim.linkurdf import LINK_URDF

from pyrosim.model import MODEL

from pyrosim.sdf   import SDF

from pyrosim.urdf  import URDF

from pyrosim.joint import JOINT

SDF_FILETYPE  = 0

URDF_FILETYPE = 1

NNDF_FILETYPE   = 2

# global availableLinkIndex

# global linkNamesToIndices

def End():

    if filetype == SDF_FILETYPE:

        sdf.Save_End_Tag(f)

    elif filetype == NNDF_FILETYPE:

        nndf.Save_End_Tag(f)
    else:
        urdf.Save_End_Tag(f)

    f.close()

def End_Model():

    model.Save_End_Tag(f)

def Get_Touch_Sensor_Value_For_Link(linkName):

    touchValue = -1.0

    desiredLinkIndex = linkNamesToIndices[linkName]

    pts = p.getContactPoints()

    for pt in pts:

        linkIndex = pt[4]

        if ( linkIndex == desiredLinkIndex ):

            touchValue = 1.0

    return touchValue

def Prepare_Link_Dictionary(bodyID):

    global linkNamesToIndices

    linkNamesToIndices = {}

    for jointIndex in range( 0 , p.getNumJoints(bodyID) ):

        jointInfo = p.getJointInfo( bodyID , jointIndex )

        jointName = jointInfo[1]

        jointName = jointName.decode("utf-8")

        jointName = jointName.split("_")

        linkName = jointName[1]

        linkNamesToIndices[linkName] = jointIndex

        if jointIndex==0:

            rootLinkName = jointName[0]

            linkNamesToIndices[rootLinkName] = -1

def Prepare_Joint_Dictionary(bodyID):

    global jointNamesToIndices

    jointNamesToIndices = {}

    for jointIndex in range( 0 , p.getNumJoints(bodyID) ):

        jointInfo = p.getJointInfo( bodyID , jointIndex )

        jointName = jointInfo[1]

        jointNamesToIndices[jointName] = jointIndex

def Prepare_To_Simulate(bodyID):

    Prepare_Link_Dictionary(bodyID)

    Prepare_Joint_Dictionary(bodyID)

def Send_Cube(name="default",pos=[0,0,0],size=[1,1,1]):

    global availableLinkIndex

    global links

    if filetype == SDF_FILETYPE:

        Start_Model(name,pos)

        link = LINK_SDF(name,pos,size)

        links.append(link)
    else:
        link = LINK_URDF(name,pos,size)

        links.append(link)

    link.Save(f)

    if filetype == SDF_FILETYPE:

        End_Model()

    linkNamesToIndices[name] = availableLinkIndex

    availableLinkIndex = availableLinkIndex + 1

def Send_Joint(name,parent,child,type,position, jointAxis):

    joint = JOINT(name,parent,child,type,position)

    joint.Save(f, jointAxis)

def Send_Motor_Neuron(name,jointName):

    f.write('    <neuron name = "' + str(name) + '" type = "motor"  jointName = "' + jointName + '" />\n')

def Send_Sensor_Neuron(name,linkName):

    f.write('    <neuron name = "' + str(name) + '" type = "sensor" linkName = "' + linkName + '" />\n')

def Send_Synapse( sourceNeuronName , targetNeuronName , weight ):

    f.write('    <synapse sourceNeuronName = "' + str(sourceNeuronName) + '" targetNeuronName = "' + str(targetNeuronName) + '" weight = "' + str(weight) + '" />\n')


def Set_Motor_For_Joint(bodyIndex,jointName,controlMode,targetPosition,maxForce):

    p.setJointMotorControl2(

        bodyIndex      = bodyIndex,

        jointIndex     = jointNamesToIndices[jointName],

        controlMode    = controlMode,

        targetPosition = targetPosition,

        force          = maxForce)

def Start_NeuralNetwork(filename):

    global filetype

    filetype = NNDF_FILETYPE

    global f

    f = open(filename,"w")

    global nndf

    nndf = NNDF()

    nndf.Save_Start_Tag(f)

def Start_SDF(filename):

    global availableLinkIndex

    availableLinkIndex = -1

    global linkNamesToIndices

    linkNamesToIndices = {}

    global filetype

    filetype = SDF_FILETYPE

    global f

    f = open(filename,"w")

    global sdf

    sdf = SDF()

    sdf.Save_Start_Tag(f)

    global links

    links = []

def Start_URDF(filename):

    global availableLinkIndex

    availableLinkIndex = -1

    global linkNamesToIndices

    linkNamesToIndices = {}

    global filetype

    filetype = URDF_FILETYPE

    global f

    f = open(filename,"w")

    global urdf

    urdf = URDF()

    urdf.Save_Start_Tag(f)

    global links

    links = []

def Start_Model(modelName,pos):

    global model

    model = MODEL(modelName,pos)

    model.Save_Start_Tag(f)


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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Constants
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import numpy

# front leg angle/motor variables
amplitude = numpy.pi / 4
frequency = 20
phaseOffset = 0

# create world variables
length = 1
width = 1
height = 1
x = 0
y = 0
z = 1

# simulation time steps
time_steps = 1000

# target angles
targetAngles = (numpy.sin(numpy.linspace(0, 2*numpy.pi, time_steps)) * numpy.pi / 4) # scale them by pi/4

# number of generation
numberOfGenerations = 10

# population size
populationSize = 10

# sleep rate
sleepRate = 1/60

numSensorNeurons = 9
numMotorNeurons = 8

motorJointRange = .2


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot
CS 206: Evolutionary Robotics

@author: tuckerparon
"""

from sensor import SENSOR
from motor import MOTOR
import pyrosim.pyrosim as pyrosim
import pybullet as p
import constants as c
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os

# robot.py
class ROBOT:
    def __init__(self, solutionID):

        # set solution ID
        self.solutionID = solutionID

        # for sensor and motor values
        self.sensors = {}
        self.motors = {}
        self.values = {}

        # set up robot
        self.robot = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robot)

        # get ready to sense and act
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

        # set up brain
        self.nn = NEURAL_NETWORK("brain" + solutionID + ".nndf")

        os.system("rm brain" + solutionID + ".nndf")

    def Prepare_To_Sense(self):

        # fill in sensor values (as value) for each link (as key)
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):

        # get sensor value for each link at each time step
        for key in self.sensors:
            self.values[t] = self.sensors[key].Get_Value(t)

    def Prepare_To_Act(self):

        # fill in motor values (as value) for each joint (as key)
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self):

        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName).encode('ASCII')
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(self.robot, desiredAngle)

    def Save_Values(self):

        # save the values
        for key in self.sensors:
            self.sensors[key].Save_Values()

        for key in self.motors:
            self.motors[key].Save_Values()


    def Think(self):
        self.nn.Update()
        #self.nn.Print()


    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robot,0)
        #print(stateOfLinkZero)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        f = open("tmp" + str(self.solutionID) + ".txt", "w")
        f.write(str(xCoordinateOfLinkZero))
        f.close()
        os.rename("tmp"+str(self.solutionID)+".txt", "fitness"+str(self.solutionID)+".txt")


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Solution
CS206: Evolutionary Robotics

@author: tuckerparon
"""

import numpy as np
import pyrosim.pyrosim as pyrosim
import random
import os
import constants as c
import time
import sys

class SOLUTION:
    def __init__(self, nextAvailableID):
        self.weights = np.random.rand(c.numSensorNeurons,c.numMotorNeurons)
        self.weights = self.weights * 2 - 1
        self.myID = nextAvailableID

    def Evaluate(self, directOrGUI):
        pass

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        os.system("python3 simulate.py " + directOrGUI + " " + str(self.myID) + " &")

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.01)
        fitnessFile = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()
        os.system("rm fitness" + str(self.myID) + ".txt")

    def Create_World(self):

        # store file
        pyrosim.Start_SDF("world.sdf")

        # create box
        pyrosim.Send_Cube(name="Box", pos=[4,4,0.5], size=[1,1,1])

        # close file
        pyrosim.End()

    def Generate_Body(self):


        # create urdf file to store description of robots body
        pyrosim.Start_URDF("body.urdf")

        # Create Torso
        pyrosim.Send_Cube(name="Torso", pos=[c.x, c.y, c.z], size=[c.length, c.width, c.height])

        # Create Legs
        pyrosim.Send_Cube(name="FrontLeg", pos=[0, .5, 0], size=[0.2, 1, 0.2])
        pyrosim.Send_Cube(name="LowerFrontLeg", pos=[0, 0, -.5], size=[.2, .2, 1])
        pyrosim.Send_Cube(name="BackLeg", pos=[0, -.5, 0], size=[0.2, 1, 0.2])
        pyrosim.Send_Cube(name="LeftLeg", pos=[-.5, 0, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Cube(name="LowerRightLeg", pos=[0, 0, -.5], size=[.2, .2, 1])
        pyrosim.Send_Cube(name="LowerLeftLeg", pos=[0, 0, -.5], size=[.2, .2, 1])
        pyrosim.Send_Cube(name="LowerBackLeg", pos=[0, 0, -.5], size=[.2, .2, 1])

        # Create Joints
        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg",type="revolute", position="0 -0.5 1", jointAxis = "1 0 0")
        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg",type="revolute", position="-0.5 0 1", jointAxis = "0 1 0")
        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg",type="revolute", position="0.5 0 1", jointAxis = "0 1 0")
        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg",type="revolute", position="0 0.5 1", jointAxis = "1 0 0")
        pyrosim.Send_Joint(name="FrontLeg_LowerFrontLeg", parent="FrontLeg", child="LowerFrontLeg",type="revolute", position="0 1 0", jointAxis = "1 0 0")
        pyrosim.Send_Joint(name="BackLeg_LowerBackLeg", parent="BackLeg", child="LowerBackLeg",type="revolute", position="0 -1 0", jointAxis = "1 0 0")
        pyrosim.Send_Joint(name="LeftLeg_LowerLeftLeg", parent="LeftLeg", child="LowerLeftLeg",type="revolute", position="-1 0 0", jointAxis = "0 1 0")
        pyrosim.Send_Joint(name="RightLeg_LowerRightLeg", parent="RightLeg", child="LowerRightLeg", type="revolute", position="1 0 0", jointAxis = "0 1 0")

        # end
        pyrosim.End()

    def Generate_Brain(self):

        # create nndf file to store brain contents
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        pyrosim.Start_NeuralNetwork("brain"+str(self.myID)+".nndf")
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="FrontLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="LeftLeg")
        pyrosim.Send_Sensor_Neuron(name=4, linkName="RightLeg")

        pyrosim.Send_Sensor_Neuron(name=5, linkName="LowerFrontLeg")
        pyrosim.Send_Sensor_Neuron(name=6, linkName="LowerBackLeg")
        pyrosim.Send_Sensor_Neuron(name=7, linkName="LowerLeftLeg")
        pyrosim.Send_Sensor_Neuron(name=8, linkName="LowerRightLeg")

        pyrosim.Send_Motor_Neuron(name=9 , jointName="Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name=10, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=11, jointName="Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name=12, jointName="Torso_RightLeg")

        pyrosim.Send_Motor_Neuron(name=13, jointName="FrontLeg_LowerFrontLeg")
        pyrosim.Send_Motor_Neuron(name=14, jointName="BackLeg_LowerBackLeg")
        pyrosim.Send_Motor_Neuron(name=15, jointName="LeftLeg_LowerLeftLeg")
        pyrosim.Send_Motor_Neuron(name=16, jointName="RightLeg_LowerRightLeg")

        # loop over the sensor neurons
        for currentRow in range(c.numSensorNeurons):
            # loop over the motor neurons
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn + c.numSensorNeurons, weight=self.weights[currentRow][currentColumn])

        # end
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons-1)
        randomColumn = random.randint(0, c.numMotorNeurons-1)
        self.weights[randomRow,randomColumn] = random.random() * 2 -1

    def Set_ID(self):
        return self.myID


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
    def __init__(self, directOrGUI, solutionID):

        self.directOrGUI = directOrGUI
        self.solutionID = solutionID

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
        self.robot.Get_Fitness()

    def __del__(self):

        # end simulation and save values
        p.disconnect()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyze
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import numpy
import matplotlib.pyplot as plt

# load in sensor values .npy file
backLegSensorValues = numpy.load('data/backLegSensorValues.npy')
frontLegSensorValues = numpy.load('data/frontLegSensorValues.npy')

# load in angle/control vectors
targetAngles =  numpy.load('data/targetAngles.npy')
backLegMotorValues =  numpy.load('data/backLegMotorValues.npy')
frontLegMotorValues =  numpy.load('data/frontLegMotorValues.npy')

# print sensor values
#print(backLegSensorValues)
#print(frontLegSensorValues)

# print angle/control values
#print(targetAngles)
#print(backLegMotorValues)
#print(frontLegMotorValues)

# plot sensor values
#plt.plot(backLegSensorValues, label='Back Leg', linewidth=1)
#plt.plot(frontLegSensorValues, label='Front Leg', linewidth=1)
#plt.plot(targetAngles, label='Angles', linewidth=1)
plt.plot(backLegMotorValues, label='Back Leg', linewidth=1)
plt.plot(frontLegMotorValues, label='Front Leg', linewidth=1)
plt.legend()
plt.show()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate
CS206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import pyrosim.pyrosim as pyrosim
import constants as c
import random

def Create_World():

    # store file
    pyrosim.Start_SDF("world.sdf")

    # create box
    pyrosim.Send_Cube(name="Box", pos=[c.x, c.y, c.z] , size = [c.length, c.width, c.height])

    # close file
    pyrosim.End()

def Generate_Body():


    # create urdf file to store description of robots body
    pyrosim.Start_URDF("body.urdf")

    # create link 0
    pyrosim.Send_Cube(name="Torso", pos=[0, 1, 1.5] , size = [c.length, c.width, c.height])

    # create link between 0 and 1
    pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [-.5, 1, 1])

    # create link 1
    pyrosim.Send_Cube(name="BackLeg", pos=[-.5, 0, -.5] , size = [c.length, c.width, c.height])

    # create link between 1 and 2
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [.5, 1, 1])

    # create link 2
    pyrosim.Send_Cube(name="FrontLeg", pos=[.5, 0, -.5] , size = [c.length, c.width, c.height])

    # end
    pyrosim.End()

def Generate_Brain():

    # create nndf file to store brain contents
    pyrosim.Start_NeuralNetwork("brain.nndf")
    print('yes')

    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1, linkName="BackLeg")
    pyrosim.Send_Sensor_Neuron(name = 2, linkName="FrontLeg")

    pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

    #pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 3 , weight = -1.0 )
    #pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 3 , weight = -1.0)
    #pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 4 , weight = -1.0)
    #pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 4 , weight = -1.0)

    # loop over sensor neurons
    for i in range(5):
        # loop over the motors
        for j in [3, 4]:
            pyrosim.Send_Synapse(sourceNeuronName = i, targetNeuronName = j, weight = random.uniform(-1.0, 1.0))

    # end
    pyrosim.End()


Create_World()
Generate_Body()
Generate_Brain()


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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor
CS 206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import numpy
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p

# motor.py
class MOTOR:
    def __init__(self, jointName):

        # set joint name and get ready to act
        self.jointName = jointName

    def Set_Value(self, robot, desiredAngle):

        # set motor values for appropriate joint
        pyrosim.Set_Motor_For_Joint(bodyIndex = robot, jointName = self.jointName, controlMode = p.POSITION_CONTROL, targetPosition = desiredAngle, maxForce = 50)


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


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sensor
CS 206: Evolutionary Robotics

@author: tuckerparon
"""

# imports
import numpy
import constants as c
import pyrosim.pyrosim as pyrosim

# sensor.py
class SENSOR:
    def __init__(self, linkName):

        # intialize linkName and link values
        self.linkName = linkName
        self.values = numpy.zeros(c.time_steps)

    def Get_Value(self, t):

        # get values at each time step and print final vectors
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName) # sensor values
        #if t == c.time_steps-1:
        #    print(self.values)

    def Save_Values(self):

        # save values
        numpy.save('data//sensorValues.npy', self.values)

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
