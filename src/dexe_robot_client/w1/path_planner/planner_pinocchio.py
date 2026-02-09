'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import pinocchio
from ...common import get_pin2real_idx, get_real2pin_idx, get_pin_idx
from .utils.robot_type import *


class Planner_Pin:

    def __init__(self,
                 robot_type: RobotType = RobotType.W1V02_1,
                 end_effector_type: EndEffectorType = None):
        robot_urdf = get_robot_urdf(robot_type=robot_type,
                                    ee_type=end_effector_type)
        self.pin_model = pinocchio.buildModelFromUrdf(robot_urdf)
        self.pin_data = self.pin_model.createData()

        self.pin2real_idx = get_pin2real_idx(robot_urdf)
        self.real2pin_idx = get_real2pin_idx(robot_urdf)

        if end_effector_type is not None and isinstance(end_effector_type,
                                                        EndEffectorType):
            self.use_ee = True
            self.ee_type = end_effector_type
            self.pin_torso_idx, self.pin_head_idx, self.pin_arml_idx, self.pin_armr_idx, self.pin_armlee_idx, self.pin_armree_idx = get_pin_idx(
                robot_urdf)
        else:
            self.use_ee = False
            self.ee_type = None
            self.pin_torso_idx, self.pin_head_idx, self.pin_arml_idx, self.pin_armr_idx = get_pin_idx(
                robot_urdf)

        self.armlee_joint_name, self.armree_joint_name = get_ee_joint_names(
            end_effector_type)

    def get_Kinematics(self,
                       real_qpos: np.ndarray = None,
                       left_ee_qpos: np.ndarray = None,
                       right_ee_qpos: np.ndarray = None,
                       compute_ee: bool = False):
        """
        This function computes the forward kinematics of the robot given the real robot joint positions.

        Args:
            real_qpos (nd.array, shape=(20, )): The joint qpos of real robot, usually in order of [torso, head, arml, armr]
            left_ee_qpos (nd.array): The joint qpos of left end effector, shape depends on the type of end effector
            right_ee_qpos (nd.array): The joint qpos of right end effector, shape depends on the type of end effector
        
        Notes:
            - If use_ee is True, the shape of left_ee_qpos and right_ee_qpos should match the end effector type.
                * BRAINCO_HAND: shape=(6, )
                * PGC_GRIPPER: shape=(2, )

        Returns:
            The tuple of transformation matrices in order of [torso, head, arml, armr]
            - torso_transform_chain: List of transformation matrices for torso links with length of 4
            - head_transform_chain: List of transformation matrices for head links with length of 2
            - arml_transform_chain: List of transformation matrices for left arm links with length of 7
            - armr_transform_chain: List of transformation matrices for right arm links with length of 7
        """
        if real_qpos is None or real_qpos.shape != (20,):
            raise ValueError(
                "real_qpos should be provided with shape of (20, )")

        if not self.use_ee and compute_ee:
            raise ValueError(
                "Planner_Pin instance was created without end effector, cannot compute end effector transforms"
            )

        if self.use_ee and compute_ee:
            if left_ee_qpos is None or right_ee_qpos is None:
                raise ValueError(
                    "left_ee_qpos and right_ee_qpos should be provided when use_ee is True"
                )
            if self.ee_type == EndEffectorType.BRAINCO_HAND:
                if left_ee_qpos.shape != (6,) or right_ee_qpos.shape != (6,):
                    raise ValueError(
                        "For BRAINCO_HAND, left_ee_qpos and right_ee_qpos should have shape of (6, )"
                    )
            elif self.ee_type == EndEffectorType.PGC_GRIPPER:
                if left_ee_qpos.shape != (1,) or right_ee_qpos.shape != (1,):
                    raise ValueError(
                        "For PGC_GRIPPER, left_ee_qpos and right_ee_qpos should have shape of (2, )"
                    )
            elif self.ee_type == EndEffectorType.LINKER_HAND:
                if left_ee_qpos.shape != (6,) or right_ee_qpos.shape != (6,):
                    raise ValueError(
                        "For LINKER_HAND, left_ee_qpos and right_ee_qpos should have shape of (6, )"
                    )
            else:
                raise ValueError("Unsupported end effector type")
            left_ee_qpos, right_ee_qpos = self._compute_ee_qpos(
                left_ee_pulse=left_ee_qpos, right_ee_pulse=right_ee_qpos)
            left_ee_qpos, right_ee_qpos = self.get_complete_ee_qpos(
                left_ee_qpos, right_ee_qpos)

            real_qpos = np.concatenate((real_qpos, left_ee_qpos, right_ee_qpos),
                                       axis=0)
        elif self.use_ee and not compute_ee:
            if self.pin_model.njoints - 1 == 40:
                real_qpos = np.concatenate((real_qpos, np.zeros(20)), axis=0)
            if self.pin_model.njoints - 1 == 24:
                real_qpos = np.concatenate((real_qpos, np.zeros(4)), axis=0)
            if self.pin_model.njoints - 1 == 42:
                real_qpos = np.concatenate((real_qpos, np.zeros(22)), axis=0)

        pin_qpos = real_qpos[self.real2pin_idx]

        pinocchio.forwardKinematics(self.pin_model, self.pin_data, pin_qpos)
        pinocchio.updateFramePlacements(self.pin_model, self.pin_data)
        transform_chain = [oMi.homogeneous for oMi in self.pin_data.oMi][1:]
        torso_transform_chain = [transform_chain[i] for i in self.pin_torso_idx]
        head_transform_chain = [transform_chain[i] for i in self.pin_head_idx]
        arml_transform_chain = [transform_chain[i] for i in self.pin_arml_idx]
        armr_transform_chain = [transform_chain[i] for i in self.pin_armr_idx]
        if self.use_ee and compute_ee:
            armlee_transform_chain, armree_transform_chain = [], []
            armlee_ID = self.pin_model.getFrameId(self.armlee_joint_name)
            armree_ID = self.pin_model.getFrameId(self.armree_joint_name)
            lee_trans = self.pin_data.oMf[armlee_ID].homogeneous
            ree_trans = self.pin_data.oMf[armree_ID].homogeneous
            armlee_transform_chain.append(lee_trans)
            armree_transform_chain.append(ree_trans)

            for i in self.pin_armlee_idx:
                armlee_transform_chain.append(transform_chain[i])
            for i in self.pin_armree_idx:
                armree_transform_chain.append(transform_chain[i])

            return torso_transform_chain, head_transform_chain, arml_transform_chain, armr_transform_chain, \
                   armlee_transform_chain, armree_transform_chain
        elif self.use_ee and not compute_ee:
            return torso_transform_chain, head_transform_chain, arml_transform_chain, armr_transform_chain

        return torso_transform_chain, head_transform_chain, arml_transform_chain, armr_transform_chain

    def get_arm_base_tf(self, real_qpos: np.ndarray = None):
        """
        This function computes the transformation matrices of left and right arm base

        Args:
            real_qpos (nd.array, shape=(20, )): The joint qpos of real robot, usually in order of [torso, head, arml, armr]

        Returns:
            The tuple of transformation matrices in order of [left arm base, right arm base]
        """
        if self.use_ee:
            if self.ee_type == EndEffectorType.BRAINCO_HAND:
                real_qpos = np.concatenate((real_qpos, np.zeros(20)), axis=0)
            elif self.ee_type == EndEffectorType.PGC_GRIPPER:
                real_qpos = np.concatenate((real_qpos, np.zeros(4)), axis=0)
            elif self.ee_type == EndEffectorType.LINKER_HAND:
                real_qpos = np.concatenate((real_qpos, np.zeros(22)), axis=0)

        pin_qpos = real_qpos[self.real2pin_idx]
        pinocchio.forwardKinematics(self.pin_model, self.pin_data, pin_qpos)
        pinocchio.updateFramePlacements(self.pin_model, self.pin_data)
        arml_base_ID = self.pin_model.getFrameId("LEFT_ARM_BASE")
        armr_base_ID = self.pin_model.getFrameId("RIGHT_ARM_BASE")
        arml_base_tf = self.pin_data.oMf[arml_base_ID].homogeneous
        armr_base_tf = self.pin_data.oMf[armr_base_ID].homogeneous
        return arml_base_tf, armr_base_tf

    def _pulse_to_radian(self, pulse: float, lower_limit: float,
                         upper_limit: float):
        radian = lower_limit + (upper_limit - lower_limit) * (pulse / 100.0)
        return radian

    def _pulse_to_position(self, pulse: float, lower_limit: float,
                           upper_limit: float):
        position = lower_limit + (upper_limit - lower_limit) * (pulse / 100.0)
        return position

    def _compute_ee_qpos(self, left_ee_pulse: np.ndarray,
                         right_ee_pulse: np.ndarray):
        if self.ee_type == EndEffectorType.BRAINCO_HAND:
            left_ee_qpos, right_ee_qpos = np.zeros(6), np.zeros(6)

            left_ee_qpos[0], left_ee_qpos[1] = self._pulse_to_radian(left_ee_pulse[0], BRAINCO_LEFT_LIMITS[0, 1], BRAINCO_LEFT_LIMITS[1, 1]), \
                                            self._pulse_to_radian(left_ee_pulse[1], BRAINCO_LEFT_LIMITS[0, 0], BRAINCO_LEFT_LIMITS[1, 0])
            left_ee_qpos[2] = self._pulse_to_radian(left_ee_pulse[2],
                                                    BRAINCO_LEFT_LIMITS[0, 2],
                                                    BRAINCO_LEFT_LIMITS[1, 2])
            left_ee_qpos[3] = self._pulse_to_radian(left_ee_pulse[3],
                                                    BRAINCO_LEFT_LIMITS[0, 4],
                                                    BRAINCO_LEFT_LIMITS[1, 4])
            left_ee_qpos[4] = self._pulse_to_radian(left_ee_pulse[4],
                                                    BRAINCO_LEFT_LIMITS[0, 6],
                                                    BRAINCO_LEFT_LIMITS[1, 6])
            left_ee_qpos[5] = self._pulse_to_radian(left_ee_pulse[5],
                                                    BRAINCO_LEFT_LIMITS[0, 8],
                                                    BRAINCO_LEFT_LIMITS[1, 8])

            right_ee_qpos[0], right_ee_qpos[1] = self._pulse_to_radian(right_ee_pulse[0], BRAINCO_RIGHT_LIMITS[0, 1], BRAINCO_RIGHT_LIMITS[1, 1]), \
                                            self._pulse_to_radian(right_ee_pulse[1], BRAINCO_RIGHT_LIMITS[0, 0], BRAINCO_RIGHT_LIMITS[1, 0])
            right_ee_qpos[2] = self._pulse_to_radian(right_ee_pulse[2],
                                                     BRAINCO_RIGHT_LIMITS[0, 2],
                                                     BRAINCO_RIGHT_LIMITS[1, 2])
            right_ee_qpos[3] = self._pulse_to_radian(right_ee_pulse[3],
                                                     BRAINCO_RIGHT_LIMITS[0, 4],
                                                     BRAINCO_RIGHT_LIMITS[1, 4])
            right_ee_qpos[4] = self._pulse_to_radian(right_ee_pulse[4],
                                                     BRAINCO_RIGHT_LIMITS[0, 6],
                                                     BRAINCO_RIGHT_LIMITS[1, 6])
            right_ee_qpos[5] = self._pulse_to_radian(right_ee_pulse[5],
                                                     BRAINCO_RIGHT_LIMITS[0, 8],
                                                     BRAINCO_RIGHT_LIMITS[1, 8])

            return left_ee_qpos, right_ee_qpos
        elif self.ee_type == EndEffectorType.LINKER_HAND:
            left_ee_qpos, right_ee_qpos = np.zeros(6), np.zeros(6)
            # pulse index order (thumb_mcp, thumb_cmc, index_mcp, middle_mcp, ring_mcp, little_mcp)
            left_ee_qpos[0], left_ee_qpos[1] = self._pulse_to_radian(left_ee_pulse[0], LINKER_LEFT_LIMITS[0,1], LINKER_LEFT_LIMITS[1,1]), \
                                            self._pulse_to_radian(left_ee_pulse[1], LINKER_LEFT_LIMITS[0,0], LINKER_LEFT_LIMITS[1,0])
            left_ee_qpos[2] = self._pulse_to_radian(left_ee_pulse[2],
                                                    LINKER_LEFT_LIMITS[0, 3],
                                                    LINKER_LEFT_LIMITS[1, 3])
            left_ee_qpos[3] = self._pulse_to_radian(left_ee_pulse[3],
                                                    LINKER_LEFT_LIMITS[0, 5],
                                                    LINKER_LEFT_LIMITS[1, 5])
            left_ee_qpos[4] = self._pulse_to_radian(left_ee_pulse[4],
                                                    LINKER_LEFT_LIMITS[0, 7],
                                                    LINKER_LEFT_LIMITS[1, 7])
            left_ee_qpos[5] = self._pulse_to_radian(left_ee_pulse[5],
                                                    LINKER_LEFT_LIMITS[0, 9],
                                                    LINKER_LEFT_LIMITS[1, 9])
            # pulse index order (thumb_mcp, thumb_cmc, index_mcp, middle_mcp, ring_mcp, little_mcp)
            right_ee_qpos[0], right_ee_qpos[1] = self._pulse_to_radian(right_ee_pulse[0], LINKER_RIGHT_LIMITS[0,1], LINKER_RIGHT_LIMITS[1,1]), \
                                            self._pulse_to_radian(right_ee_pulse[1], LINKER_RIGHT_LIMITS[0,0], LINKER_RIGHT_LIMITS[1,0])
            right_ee_qpos[2] = self._pulse_to_radian(right_ee_pulse[2],
                                                     LINKER_RIGHT_LIMITS[0, 3],
                                                     LINKER_RIGHT_LIMITS[1, 3])
            right_ee_qpos[3] = self._pulse_to_radian(right_ee_pulse[3],
                                                     LINKER_RIGHT_LIMITS[0, 5],
                                                     LINKER_RIGHT_LIMITS[1, 5])
            right_ee_qpos[4] = self._pulse_to_radian(right_ee_pulse[4],
                                                     LINKER_RIGHT_LIMITS[0, 7],
                                                     LINKER_RIGHT_LIMITS[1, 7])
            right_ee_qpos[5] = self._pulse_to_radian(right_ee_pulse[5],
                                                     LINKER_RIGHT_LIMITS[0, 9],
                                                     LINKER_RIGHT_LIMITS[1, 9])

            return left_ee_qpos, right_ee_qpos
        elif self.ee_type == EndEffectorType.PGC_GRIPPER:
            left_ee_qpos, right_ee_qpos = np.zeros(1), np.zeros(1)

            left_ee_qpos[0] = self._pulse_to_position(left_ee_pulse[0],
                                                      PGC_LEFT_LIMITS[0, 0],
                                                      PGC_LEFT_LIMITS[1, 0])
            right_ee_qpos[0] = self._pulse_to_position(right_ee_pulse[0],
                                                       PGC_RIGHT_LIMITS[0, 0],
                                                       PGC_RIGHT_LIMITS[1, 0])

            return left_ee_qpos, right_ee_qpos
        else:
            raise ValueError("Unsupported end effector type")

    def _compute_ee_mimic_qpos(self, parent_qpos: float, multiplier: float,
                               offset: float, lower_limit: float,
                               upper_limit: float):
        mimic_qpos = parent_qpos * multiplier + offset
        mimic_qpos = np.clip(mimic_qpos, lower_limit, upper_limit)
        return mimic_qpos

    def get_complete_ee_qpos(self, armlee_qpos: np.ndarray,
                             armree_qpos: np.ndarray):
        if self.ee_type == EndEffectorType.BRAINCO_HAND:
            arml_thumb_cmc, arml_thumb_mcp = armlee_qpos[1], armlee_qpos[0]
            # arml_thumb_cmc, arml_thumb_mcp = armlee_qpos[0], armlee_qpos[1]
            arml_index_mcp = armlee_qpos[2]
            arml_index_pip = self._compute_ee_mimic_qpos(
                arml_index_mcp, 1.2, 0.1937, BRAINCO_LEFT_LIMITS[0, 3],
                BRAINCO_LEFT_LIMITS[1, 3])
            arml_middle_mcp = armlee_qpos[3]
            arml_middle_pip = self._compute_ee_mimic_qpos(
                arml_middle_mcp, 1.2, 0.1833, BRAINCO_LEFT_LIMITS[0, 5],
                BRAINCO_LEFT_LIMITS[1, 5])
            arml_ring_mcp = armlee_qpos[4]
            arml_ring_pip = self._compute_ee_mimic_qpos(
                arml_ring_mcp, 1.2, 0.1658, BRAINCO_LEFT_LIMITS[0, 7],
                BRAINCO_LEFT_LIMITS[1, 7])
            arml_little_mcp = armlee_qpos[5]
            arml_little_pip = self._compute_ee_mimic_qpos(
                arml_little_mcp, 1.2, 0.2025, BRAINCO_LEFT_LIMITS[0, 9],
                BRAINCO_LEFT_LIMITS[1, 9])

            armr_thumb_cmc, armr_thumb_mcp = armree_qpos[1], armree_qpos[0]
            # armr_thumb_cmc, armr_thumb_mcp = armree_qpos[0], armree_qpos[1]
            armr_index_mcp = armree_qpos[2]
            armr_index_pip = self._compute_ee_mimic_qpos(
                armr_index_mcp, 1.2, 0.1937, BRAINCO_RIGHT_LIMITS[0, 3],
                BRAINCO_RIGHT_LIMITS[1, 3])
            armr_middle_mcp = armree_qpos[3]
            armr_middle_pip = self._compute_ee_mimic_qpos(
                armr_middle_mcp, 1.2, 0.1833, BRAINCO_RIGHT_LIMITS[0, 5],
                BRAINCO_RIGHT_LIMITS[1, 5])
            armr_ring_mcp = armree_qpos[4]
            armr_ring_pip = self._compute_ee_mimic_qpos(
                armr_ring_mcp, 1.2, 0.1658, BRAINCO_RIGHT_LIMITS[0, 7],
                BRAINCO_RIGHT_LIMITS[1, 7])
            armr_little_mcp = armree_qpos[5]
            armr_little_pip = self._compute_ee_mimic_qpos(
                armr_little_mcp, 1.2, 0.2025, BRAINCO_RIGHT_LIMITS[0, 9],
                BRAINCO_RIGHT_LIMITS[1, 9])

            complete_armlee_qpos = np.array([
                arml_thumb_cmc, arml_thumb_mcp, arml_index_mcp, arml_index_pip,
                arml_middle_mcp, arml_middle_pip, arml_ring_mcp, arml_ring_pip,
                arml_little_mcp, arml_little_pip
            ])
            complete_armree_qpos = np.array([
                armr_thumb_cmc, armr_thumb_mcp, armr_index_mcp, armr_index_pip,
                armr_middle_mcp, armr_middle_pip, armr_ring_mcp, armr_ring_pip,
                armr_little_mcp, armr_little_pip
            ])
            return complete_armlee_qpos, complete_armree_qpos
        elif self.ee_type == EndEffectorType.LINKER_HAND:
            arml_thumb_cmc, arml_thumb_mcp = armlee_qpos[1], armlee_qpos[0]
            # arml_thumb_cmc, arml_thumb_mcp = armlee_qpos[0], armlee_qpos[1]
            arml_thumb_pip = self._compute_ee_mimic_qpos(
                arml_thumb_mcp, 0.611762, 0.612611, LINKER_LEFT_LIMITS[0, 2],
                LINKER_LEFT_LIMITS[1, 2])
            arml_index_mcp = armlee_qpos[2]
            arml_index_pip = self._compute_ee_mimic_qpos(
                arml_index_mcp, 0.976341, 0.0, LINKER_LEFT_LIMITS[0, 4],
                LINKER_LEFT_LIMITS[1, 4])
            arml_middle_mcp = armlee_qpos[3]
            arml_middle_pip = self._compute_ee_mimic_qpos(
                arml_middle_mcp, 1.001797, 0.0, LINKER_LEFT_LIMITS[0, 6],
                LINKER_LEFT_LIMITS[1, 6])
            arml_ring_mcp = armlee_qpos[4]
            arml_ring_pip = self._compute_ee_mimic_qpos(
                arml_ring_mcp, 0.982213, 0.0, LINKER_LEFT_LIMITS[0, 8],
                LINKER_LEFT_LIMITS[1, 8])
            arml_little_mcp = armlee_qpos[5]
            arml_little_pip = self._compute_ee_mimic_qpos(
                arml_little_mcp, 0.974884, 0.0, LINKER_LEFT_LIMITS[0, 10],
                LINKER_LEFT_LIMITS[1, 10])

            armr_thumb_cmc, armr_thumb_mcp = armree_qpos[1], armree_qpos[0]
            # armr_thumb_cmc, armr_thumb_mcp = armree_qpos[0], armree_qpos[1]
            armr_thumb_pip = self._compute_ee_mimic_qpos(
                armr_thumb_mcp, 0.611762, 0.612611, LINKER_RIGHT_LIMITS[0, 2],
                LINKER_RIGHT_LIMITS[1, 2])
            armr_index_mcp = armree_qpos[2]
            armr_index_pip = self._compute_ee_mimic_qpos(
                armr_index_mcp, 0.976341, 0.0, LINKER_RIGHT_LIMITS[0, 4],
                LINKER_RIGHT_LIMITS[1, 4])
            armr_middle_mcp = armree_qpos[3]
            armr_middle_pip = self._compute_ee_mimic_qpos(
                armr_middle_mcp, 1.001797, 0.0, LINKER_RIGHT_LIMITS[0, 6],
                LINKER_RIGHT_LIMITS[1, 6])
            armr_ring_mcp = armree_qpos[4]
            armr_ring_pip = self._compute_ee_mimic_qpos(
                armr_ring_mcp, 0.982213, 0.0, LINKER_RIGHT_LIMITS[0, 8],
                LINKER_RIGHT_LIMITS[1, 8])
            armr_little_mcp = armree_qpos[5]
            armr_little_pip = self._compute_ee_mimic_qpos(
                armr_little_mcp, 0.974884, 0.0, LINKER_RIGHT_LIMITS[0, 10],
                LINKER_RIGHT_LIMITS[1, 10])

            complete_armlee_qpos = np.array([
                arml_thumb_cmc, arml_thumb_mcp, arml_thumb_pip, arml_index_mcp,
                arml_index_pip, arml_middle_mcp, arml_middle_pip, arml_ring_mcp,
                arml_ring_pip, arml_little_mcp, arml_little_pip
            ])
            complete_armree_qpos = np.array([
                armr_thumb_cmc, armr_thumb_mcp, armr_thumb_pip, armr_index_mcp,
                armr_index_pip, armr_middle_mcp, armr_middle_pip, armr_ring_mcp,
                armr_ring_pip, armr_little_mcp, armr_little_pip
            ])
            return complete_armlee_qpos, complete_armree_qpos
        elif self.ee_type == EndEffectorType.PGC_GRIPPER:
            arml_finger1 = armlee_qpos[0]
            arml_finger2 = self._compute_ee_mimic_qpos(arml_finger1, 1.0, 0.0,
                                                       PGC_LEFT_LIMITS[0, 1],
                                                       PGC_LEFT_LIMITS[1, 1])

            armr_finger1 = armree_qpos[0]
            armr_finger2 = self._compute_ee_mimic_qpos(armr_finger1, 1.0, 0.0,
                                                       PGC_RIGHT_LIMITS[0, 1],
                                                       PGC_RIGHT_LIMITS[1, 1])

            complete_armlee_qpos = np.array([arml_finger1, arml_finger2])
            complete_armree_qpos = np.array([armr_finger1, armr_finger2])

            return complete_armlee_qpos, complete_armree_qpos
        else:
            raise ValueError("Unsupported end effector type")
