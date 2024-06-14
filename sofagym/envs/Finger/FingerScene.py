# -*- coding: utf-8 -*-
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))
import Sofa.Core
import Sofa.constants.Key as Key
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
from splib3.animation import AnimationManagerController
from stlib3.scene import MainHeader, ContactHeader
from FingerToolbox import rewardShaper, goalSetter
import os
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

class FingerController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.root = args[1]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.plus:
            displacement += 0.5
            if displacement > 62:
                displacement = 62

        elif e["key"] == Key.minus:
            displacement -= 0.5
            if displacement < 0:
                displacement = 0
        print(displacement)
        print(self.root.Finger.ElasticMaterialObject.dofs.position[149])
        self.cable.CableConstraint.value = [displacement]

def add_goal_node(root):
    goal = root.addChild("Goal")
    goal.addObject('VisualStyle', displayFlags="showCollisionModels")
    goal_mo = goal.addObject('MechanicalObject', name='GoalMO', showObject=True, drawMode="1", showObjectScale=3,
                             showColor=[0, 1, 0, 1], position=[30.0, 30.0, 30.0])
    return goal_mo

def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-0.0, 0.0, 0.0, 10.0, 15.0, 60.0], pullPointLocation=[0.0, 19.559, 19.177]):
    finger = parentNode.addChild(name)
    eobject = ElasticMaterialObject(finger,
                                    volumeMeshFileName=path+"zander_gmsh.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=0.5,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName=path+"zander_gmsh.stl",
                                    rotation=rotation,
                                    translation=translation)
    finger.addChild(eobject)

    FixedBox(eobject, atPositions=fixingBox, doVisualization=True)

    cable = PullingCable(eobject,
                         "PullingCable",
                         pullPointLocation=pullPointLocation,
                         rotation=rotation,
                         translation=translation,
                         cableGeometry=loadPointListFromFile(path+"cable_test_2.json"))

    eobject.addObject(FingerController(cable, parentNode))

    CollisionMesh(eobject, name="CollisionMesh",
                  surfaceMeshFileName=path+"zander_gmsh.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1, 2])

    CollisionMesh(eobject, name="CollisionMeshAuto1",
                  surfaceMeshFileName=path+"group_left.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1])

    CollisionMesh(eobject, name="CollisionMeshAuto2",
                  surfaceMeshFileName=path+"group_right.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[2])

    return finger


# def createScene(rootNode, config={"source": [-100.0, -25, 100],
#                                   "target": [30, -25, 100],
#                                   "goalPos": [0,0,0],
#                                   "dt": 0.01}, mode='simu_and_visu'):
#     # Chose the mode: visualization or computations (or both)
#     visu, simu = False, False
#     if 'visu' in mode:
#         visu = True
#     if 'simu' in mode:
#         simu = True

#     rootNode.addObject("RequiredPlugin", name="SoftRobots")
#     rootNode.addObject("RequiredPlugin", name="SofaSparseSolver")
#     rootNode.addObject("RequiredPlugin", name="SofaPreconditioner")
#     rootNode.addObject("RequiredPlugin", name="SofaPython3")
#     rootNode.addObject('RequiredPlugin', name='BeamAdapter')
#     rootNode.addObject('RequiredPlugin', name='SofaOpenglVisual')
#     rootNode.addObject('RequiredPlugin', name="SofaMiscCollision")
#     rootNode.addObject("RequiredPlugin", name="SofaBoundaryCondition")
#     rootNode.addObject("RequiredPlugin", name="SofaConstraint")
#     rootNode.addObject("RequiredPlugin", name="SofaEngine")
#     rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
#     rootNode.addObject('RequiredPlugin', name='SofaLoader')
#     rootNode.addObject('RequiredPlugin', name="SofaSimpleFem")

#     if visu:
#         source = config["source"]
#         target = config["target"]
#         rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels '
#                                                        'hideMappings hideForceFields showWireframe')
#         # rootNode.addObject("LightManager")

#         spotLoc = [2*source[0], 0, 0]
#         # rootNode.addObject("SpotLight", position=spotLoc, direction=[-np.sign(source[0]), 0.0, 0.0])
#         rootNode.addObject("InteractiveCamera", name='camera', position=source, lookAt=target, zFar=500)
#         rootNode.addObject('BackgroundSetting', color=[1, 1, 1, 1])
#     if simu:
#         rootNode.addObject('DefaultPipeline')
#         rootNode.addObject('FreeMotionAnimationLoop')
#         rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
#         rootNode.addObject('BruteForceDetection')
#         rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(0.3), name='Response',
#                            response='FrictionContactConstraint')
#         rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)

#         rootNode.addObject(AnimationManagerController(rootNode, name="AnimationManager"))

        

#     rootNode.dt.value = config["dt"]
#     MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
#     ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)

#     simulation = rootNode.addChild("Simulation")

#     # if simu:
#     #     simulation.addObject('EulerImplicitSolver', name='odesolver', firstOrder="0", rayleighMass="0.1",
#     #                          rayleighStiffness="0.1")
#     #     simulation.addObject('EigenSimplicialLDLT',template='CompressedRowSparseMatrixd', name='linearSolver')
#     #     simulation.addObject('GenericConstraintCorrection', solverName="@linearSolver")

#     finger = Finger(simulation, translation=[1.0, 0.0, 0.0])
#     rootNode.finger = finger


#     goal_mo = add_goal_node(rootNode)

#     rootNode.addObject(rewardShaper(name="Reward", rootNode=rootNode, goalPos=config['goalPos']))
#     rootNode.addObject(goalSetter(name="GoalSetter", goalMO=goal_mo, goalPos=config['goalPos']))

#     return rootNode

def createScene(rootNode, config={"source": [-100.0, -25, 100],
                                  "target": [30, -25, 100],
                                  "goalPos": [0,0,0],
                                  "dt": 0.01}, mode='simu_and_visu'):
    # rootNode.addObject("RequiredPlugin", name="Sofa.GL.Component.Rendering3D")
    # rootNode.addObject("RequiredPlugin", name="Sofa.GL.Component.Shader")

    visu, simu = False, False
    if 'visu' in mode:
        visu = True
    if 'simu' in mode:
        simu = True

    rootNode.addObject("RequiredPlugin", name="SoftRobots")
    rootNode.addObject("RequiredPlugin", name="SofaSparseSolver")
    rootNode.addObject("RequiredPlugin", name="SofaPreconditioner")
    rootNode.addObject("RequiredPlugin", name="SofaPython3")
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SofaOpenglVisual')
    rootNode.addObject('RequiredPlugin', name="SofaMiscCollision")
    rootNode.addObject("RequiredPlugin", name="SofaBoundaryCondition")
    rootNode.addObject("RequiredPlugin", name="SofaConstraint")
    rootNode.addObject("RequiredPlugin", name="SofaEngine")
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaLoader')
    rootNode.addObject('RequiredPlugin', name="SofaSimpleFem")


    if visu:
        source = config["source"]
        target = config["target"]
        rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels '
                                                       'hideMappings hideForceFields showWireframe')
        rootNode.addObject("LightManager")
        
        spotLoc = [10*source[0], 0, 0]
        dir = [-np.sign(source[0]), 0.0, 0.0]
        rootNode.addObject("DirectionalLight", direction=[1,0,0])
        rootNode.addObject("DirectionalLight", direction=[0,1,0])
        rootNode.addObject("DirectionalLight", direction=[0,0,1])
        rootNode.addObject("InteractiveCamera", name='camera', position=source, lookAt=target, zFar=1000)
        rootNode.addObject('BackgroundSetting', color=[1, 1, 1, 1])
    if simu:
        rootNode.addObject('DefaultPipeline')
        rootNode.addObject('FreeMotionAnimationLoop')
        rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
        rootNode.addObject('BruteForceDetection')
        rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(0.3), name='Response',
                           response='FrictionContactConstraint')
        rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)

        #rootNode.addObject(AnimationManagerController(rootNode))

        #rootNode.gravity.value = [0., -9810., 0.]

    rootNode.addObject(AnimationManagerController(rootNode, name="AnimationManager"))
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)
    #rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    simulation = rootNode.addChild("Simulation")

    # if simu:
    #     simulation.addObject('EulerImplicitSolver', name='odesolver', firstOrder="0", rayleighMass="0.1",
    #                          rayleighStiffness="0.1")
    #     simulation.addObject('EigenSimplicialLDLT',template='CompressedRowSparseMatrixd', name='linearSolver')
    #     simulation.addObject('GenericConstraintCorrection', solverName="@linearSolver")

    finger = Finger(simulation, translation=[1.0, 0.0, 0.0])
    rootNode.finger = finger


    #Finger(rootNode, translation=[1.0, 0.0, 0.0])
    goal_mo = add_goal_node(rootNode)
    rootNode.addObject(rewardShaper(name="Reward", rootNode=rootNode, goalPos=config['goalPos']))
    rootNode.addObject(goalSetter(name="GoalSetter", goalMO=goal_mo, goalPos=config['goalPos']))

    return rootNode