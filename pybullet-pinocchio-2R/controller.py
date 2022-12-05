import numpy as np

import pinocchio as pin

KP = 50
KD = 5

def JointPositionControl(robotModel, q, qdot, qdes):
    """
    Joint position controller implemented via inverse dynamics + PD
    """

    qddot_des = KP*(qdes-q) + KD*(-qdot)    
    torque = pin.rnea(robotModel.model, robotModel.data, q, qdot, qddot_des)

    return torque


def computeEEpose(robotModel, q):
    pin.forwardKinematics(robotModel.model, robotModel.data, q)
    pin.updateFramePlacements(robotModel.model, robotModel.data)
    EEid = robotModel.model.getFrameId("end_effector")
    return robotModel.data.oMf[EEid]
