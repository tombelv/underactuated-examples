import pybullet as pb
import time
import numpy as np
import pinocchio as pin

import sim_utils
import controller

simDT = 0.005 # simulation timestep
simTime = 10 # total simulation time in seconds

robotID, robotModel = sim_utils.simulationSetup(simDT)

nDof = 2

# we are going to consider both revolute joints, so we fill the whole joint indices list
jointIndices = range(nDof)


q, qdot = sim_utils.getState(robotID, jointIndices) 
# in general we need to call this to compute all the kinematic and dynamic quantities (see pinocchio docs)
pin.computeAllTerms(robotModel.model, robotModel.data, q, qdot)

# set a desired joint configuration
qdes = np.array([np.pi/2,-np.pi/2])


input("press ENTER to START the simulation:")

for i in range(int(simTime/simDT)):

    # read the current joint state from the simulator
    q, qdot = sim_utils.getState(robotID, jointIndices)    

    print("error = ", q-qdes)
    print(controller.computeEEpose(robotModel, q))

    # compute the feedback torque command
    tau = controller.JointPositionControl(robotModel, q, qdot, qdes)

    # send the torque command to the simulator
    pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces = tau)

    # advance the simulation one step
    pb.stepSimulation()
    time.sleep(simDT)

    
input("press ENTER to END the simulation:")

pb.disconnect()




