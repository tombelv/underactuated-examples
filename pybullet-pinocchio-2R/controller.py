import numpy as np

import pinocchio as pin

def JointPositionControl(robotModel, q, qdot, qdes):
    """
    Joint position controller implemented via inverse dynamics + PD
    """

    KP = 50
    KD = 5
    
    qddot_des = KP*(qdes-q) + KD*(-qdot)    
    torque = pin.rnea(robotModel.model, robotModel.data, q, qdot, qddot_des)

    return torque

def EePositionController(robotModel, q, qdot, ee_des):

    KP = np.array([[50, 0],[0, 50]])
    KD = np.array([[5, 0],[0, 5]])

    # compute end effector position
    ee_homo = computeEEpose(robotModel, q)
    ee_pos = ee_homo.translation[1:3]

    ### compute the geometric Jacobian J and dJ/dt
    # update the joint placements according to the current joint configuration
    pin.forwardKinematics(robotModel.model, robotModel.data, q)
    # Computes the full model Jacobian
    pin.computeJointJacobians(robotModel.model, robotModel.data, q)
    # updates the position of each frame contained in the mode
    pin.updateFramePlacements(robotModel.model, robotModel.data)
    # get ID of the end effector
    EEid = robotModel.model.getFrameId("end_effector")
    # Jacobian of the end effector and its derivative
    Jee = pin.getFrameJacobian(robotModel.model, robotModel.data, EEid, pin.ReferenceFrame.LOCAL)[1:3]
    JeeDot = pin.computeJointJacobiansTimeVariation(robotModel.model, robotModel.data, q, qdot)[1:3]
    

    # compute the desired accelleration of the end effector
    ee_acc = KP@(ee_des - ee_pos) + KD@(-np.dot(Jee, qdot))

    # desired joint acceleration
    qddot_des = np.linalg.pinv(Jee)@(ee_acc - JeeDot@qdot)

    return pin.rnea(robotModel.model, robotModel.data, q, qdot, qddot_des)


def computeEEpose(robotModel, q):
    pin.forwardKinematics(robotModel.model, robotModel.data, q)
    pin.updateFramePlacements(robotModel.model, robotModel.data)
    EEid = robotModel.model.getFrameId("end_effector")
    return robotModel.data.oMf[EEid]
