import numpy as np

import pinocchio as pin

def JointPositionControl(robotModel, q, qdot, qdes):
    """
    Joint position controller implemented via inverse dynamics + PD
    """

    KP = np.diag([50,50])
    KD = np.diag([10,10])
    
    qddot_des = KP@(qdes-q) + KD@(-qdot)    
    torque = pin.rnea(robotModel.model, robotModel.data, q, qdot, qddot_des)

    return torque

def EePositionController(robotModel, q, qdot, ee_des):
    """
    Position control for the end effector using Jacobian pseudo-inverse
    and synamic inversion (via RNEA)
    """

    KP = np.diag([50,50])
    KD = np.diag([10,10])

    # compute end effector position
    ee_pos = computeEEpose(robotModel, q).translation[1:3]

    ### compute the geometric Jacobian J and dJ/dt
    # update the joint placements according to the current joint configuration
    pin.forwardKinematics(robotModel.model, robotModel.data, q)
    # Computes the full model Jacobian
    pin.computeJointJacobians(robotModel.model, robotModel.data, q)
    pin.computeJointJacobiansTimeVariation(robotModel.model, robotModel.data, q, qdot)
    # updates the position of each frame contained in the mode
    pin.updateFramePlacements(robotModel.model, robotModel.data)
    # get ID of the end effector
    EEid = robotModel.model.getFrameId("end_effector")
    # Jacobian of the end effector and its derivative
    Jee = pin.getFrameJacobian(robotModel.model, robotModel.data, EEid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[1:3]
    JeeDot = pin.getFrameJacobianTimeVariation(robotModel.model, robotModel.data, EEid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[1:3]
    

    # compute the desired accelleration of the end effector
    ee_acc = KP@(ee_des - ee_pos) + KD@(-Jee.dot(qdot))

    # desired joint acceleration
    qddot_des = np.linalg.pinv(Jee)@(ee_acc - JeeDot@qdot)

    return pin.rnea(robotModel.model, robotModel.data, q, qdot, qddot_des)


def computeEEpose(robotModel, q):
    """
    Computes the end-effector pose as the relative (fictitius) link in the urdf.
    Pinocchio is used to first update the kinematics data and then extract the 
    proper frame pose from data
    """
    pin.forwardKinematics(robotModel.model, robotModel.data, q)
    pin.updateFramePlacements(robotModel.model, robotModel.data)
    # Find the index of the EE frame in the pinocchio model
    EEid = robotModel.model.getFrameId("end_effector")

    return robotModel.data.oMf[EEid]
