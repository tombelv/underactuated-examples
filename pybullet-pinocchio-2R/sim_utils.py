
import pybullet as pb
import pybullet_data

import pinocchio as pin

import numpy as np

import os
from dataclasses import dataclass



def simulationSetup(simDT):

    physicsClient = pb.connect(pb.GUI) 
    # physicsClient = pb.connect(pb.DIRECT) # for non-graphical version
    pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 

    # set camera angle and position
    pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=-15, cameraTargetPosition=[0,0,0.5])
    # disable all additional GUI
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI,0)

    # set gravity and simulation timestep
    pb.setGravity(0,0,-9.81)
    pb.setTimeStep(simDT)
    pb.setRealTimeSimulation(False)

    # load the ground model
    planeId = pb.loadURDF("plane.urdf")

    # load the robot model setting initial position and orientation of the base link
    startPos = [0,0,0.5]
    startOrientation = pb.getQuaternionFromEuler([0,0,0])

    dir_path = os.path.dirname(os.path.realpath(__file__)) + "/"
    urdf_filename = dir_path + "double_pendulum_simple.urdf"

    robotID = pb.loadURDF(urdf_filename, startPos, startOrientation, useFixedBase=1) # note that here we fix the base link

    nDof = pb.getNumJoints(robotID)

    # since pybullet defaults to velocity control set to 0 for all joints, to do
    # force control or to make joints passive we need to deactivate it by setting the maximum force to zero
    pb.setJointMotorControlArray(
        robotID, 
        jointIndices=range(nDof),
        controlMode = pb.VELOCITY_CONTROL,
        forces = [0]*nDof)

    # step simulation one time for initialization
    pb.stepSimulation()

    # build the pinocchio model 
    pin_model = pin.buildModelFromUrdf(urdf_filename)
    pin_data = pin_model.createData()

    robotModel = RobotModel(pin_model, pin_data)

    return robotID, robotModel



def getState(robotID, jointList):

    currentJointStates = pb.getJointStates(robotID, jointList)
    q = np.array([currentJointStates[jointIdx][0] for jointIdx in jointList])
    qdot = np.array([currentJointStates[jointIdx][1] for jointIdx in jointList])

    return q, qdot


@dataclass
class RobotModel():
    model: ...
    data: ...
