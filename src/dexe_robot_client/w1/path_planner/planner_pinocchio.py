# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import pinocchio
from ...common import get_pin2real_idx, get_real2pin_idx, get_pin_idx
from .utils.robot_type import *


class Planner_Pin:

    def __init__(self, robot_type: RobotType = RobotType.W1V02_1, end_effector_type: EndEffectorType = None):
        robot_urdf = get_robot_urdf(robot_type=robot_type)
        self.pin_model = pinocchio.buildModelFromUrdf(robot_urdf)
        self.pin_data = self.pin_model.createData()

        self.pin2real_idx = get_pin2real_idx(robot_urdf)
        self.real2pin_idx = get_real2pin_idx(robot_urdf)

        self.pin_torso_idx, self.pin_head_idx, self.pin_arml_idx, self.pin_armr_idx = get_pin_idx(robot_urdf)

        if end_effector_type is not None and isinstance(end_effector_type, EndEffectorType):
            self.use_ee = True
            self.ee_type = end_effector_type
        else:
            self.use_ee = False
            self.ee_type = None

        self.armlee_joint_name, self.armree_joint_name = get_ee_joint_names()

    def get_Kinematics(self, real_qpos: np.ndarray = None):
        """
        This function computes the forward kinematics of the robot given the real robot joint positions.

        Args:
            real_qpos (nd.array, shape=(20, )): The joint qpos of real robot, usually in order of [torso, head, arml, armr]
        
        Returns:
            The tuple of transformation matrices in order of [torso, head, arml, armr]
            - torso_transform_chain: List of transformation matrices for torso links with length of 4
            - head_transform_chain: List of transformation matrices for head links with length of 2
            - arml_transform_chain: List of transformation matrices for left arm links with length of 7 (8 if use_ee)
            - armr_transform_chain: List of transformation matrices for right arm links with length of 7 (8 if use_ee)
        """

        pin_qpos = real_qpos[self.real2pin_idx]
        pinocchio.forwardKinematics(self.pin_model, self.pin_data, pin_qpos)
        pinocchio.updateFramePlacements(self.pin_model, self.pin_data)
        transform_chain = [oMi.homogeneous for oMi in self.pin_data.oMi][1:]
        torso_transform_chain = transform_chain[self.pin_torso_idx[0]:self.pin_torso_idx[1]+1]
        head_transform_chain = transform_chain[self.pin_head_idx[0]:self.pin_head_idx[1]+1]
        arml_transform_chain = transform_chain[self.pin_arml_idx[0]:self.pin_arml_idx[1]+1]
        armr_transform_chain = transform_chain[self.pin_armr_idx[0]:self.pin_armr_idx[1]+1]
        if self.use_ee:
            armlee_ID = self.pin_model.getFrameId(self.armlee_joint_name)
            armree_ID = self.pin_model.getFrameId(self.armree_joint_name)
            lee_trans = self.pin_data.oMf[armlee_ID].homogeneous
            ree_trans = self.pin_data.oMf[armree_ID].homogeneous
            arml_transform_chain.append(lee_trans)
            armr_transform_chain.append(ree_trans)

        return torso_transform_chain, head_transform_chain, arml_transform_chain, armr_transform_chain


    def get_arm_base_tf(self, real_qpos: np.ndarray = None):
        """
        This function computes the transformation matrices of left and right arm base

        Args:
            real_qpos (nd.array, shape=(20, )): The joint qpos of real robot, usually in order of [torso, head, arml, armr]

        Returns:
            The tuple of transformation matrices in order of [left arm base, right arm base]
        """
        pin_qpos = real_qpos[self.real2pin_idx]
        pinocchio.forwardKinematics(self.pin_model, self.pin_data, pin_qpos)
        pinocchio.updateFramePlacements(self.pin_model, self.pin_data)
        arml_base_ID = self.pin_model.getFrameId("LRFT_ARM_BASE")
        armr_base_ID = self.pin_model.getFrameId("RIGHT_ARM_BASE")
        arml_base_tf = self.pin_data.oMf[arml_base_ID].homogeneous
        armr_base_tf = self.pin_data.oMf[armr_base_ID].homogeneous
        return arml_base_tf, armr_base_tf
