# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import json
import numpy as np
from ...end_effector_client.end_effector_type import EndEffectorType
from ...common import get_logger, DATA_DIR, W1ArmType, W1Version


class W1Params:

    def __init__(self,
                 arm_type: W1ArmType,
                 w1_version: W1Version = W1Version.V2,
                 end_effector_type: EndEffectorType = None):
        self.logger = get_logger()

        self._arm_type = arm_type

        self._w1_version = w1_version
        if self._w1_version != W1Version.V2:
            raise ValueError(
                f"W1Version {self._w1_version.name} are not support.")

        self.arm_ee_dofs = None
        self._end_effector_type = end_effector_type
        if self._end_effector_type is None:
            self.logger.info(f"No movable end effector.")
        elif self._end_effector_type == EndEffectorType.BRAINCO_HAND:
            self.arm_ee_dofs = 6
        elif self._end_effector_type == EndEffectorType.LINKER_HAND:
            self.arm_ee_dofs = 6
        elif self._end_effector_type == EndEffectorType.PGC_GRIPPER:
            self.arm_ee_dofs = 1
        else:
            raise ValueError(
                f"Unsupport end effector type: {self._end_effector_type.name}.")

        self.torso_dofs = 4
        self.head_dofs = 2
        self.arm_dofs = 7
        self.full_body_dofs = 20

        # IndexManager
        self.torso_index = list(range(4))
        self.head_index = [4, 5]
        self.left_arm_index = list(range(6, 13))
        self.right_arm_index = list(range(13, 20))
        self.full_body_index = list(range(20))

        self.index_infos = {
            "torso": {
                "dofs": self.torso_dofs,
                "index": self.torso_index
            },
            "head": {
                "dofs": self.head_dofs,
                "index": self.head_index
            },
            "left_arm": {
                "dofs": self.arm_dofs,
                "index": self.left_arm_index
            },
            "right_arm": {
                "dofs": self.arm_dofs,
                "index": self.right_arm_index
            },
            "full_body": {
                "dofs": self.full_body_dofs,
                "index": self.full_body_index
            }
        }

        self.names = ("torso", "head", "left_arm", "right_arm", "full_body")
        self.arm_names = ["left_arm", "right_arm"]
        self.arm_ee_names = ["left_arm_ee", "right_arm_ee"]

        # Kinematics Params
        self.torso_ub = np.deg2rad([90., 155., 90., 170.])
        self.torso_lb = np.deg2rad([-90., -155., -110., -170.])
        self.head_ub = np.deg2rad([90., 25.])
        self.head_lb = np.deg2rad([-90., -45.])

        if self._arm_type == W1ArmType.ANTHROPOMORPHIC:
            self.left_arm_ub = np.deg2rad(
                [170., 90., 170., 90., 170., 45., 60.])
            self.left_arm_lb = np.deg2rad(
                [-170., -120., -170., -135., -170., -45., -90.])
            self.right_arm_ub = np.deg2rad(
                [170., 120., 170., 135., 170., 45., 90.])
            self.right_arm_lb = np.deg2rad(
                [-170., -90., -170., -90., -170., -45., -60.])
        else:
            self.left_arm_ub = np.deg2rad(
                [170., 90., 170., 90., 170., 90., 170.])
            self.left_arm_lb = np.deg2rad(
                [-170., -120., -170., -135., -170., -90., -170.])
            self.right_arm_ub = np.deg2rad(
                [170., 120., 170., 135., 170., 90., 170.])
            self.right_arm_lb = np.deg2rad(
                [-170., -90., -170., -90., -170., -90., -170.])

        self.qpos_ub = np.r_[self.torso_ub, self.head_ub, self.left_arm_ub,
                             self.right_arm_ub]
        self.qpos_lb = np.r_[self.torso_lb, self.head_lb, self.left_arm_lb,
                             self.right_arm_lb]

        # yapf: disable
        self.default_qvel = np.array([0.3] * self.full_body_dofs)
        self.min_qvel = np.array([0.05] * self.full_body_dofs)
        self.max_qvel = np.array([1., 1., 1., 1., 6.28, 6.28,
                                  6.28, 6.28, 6.28, 6.28, 6.28, 6.28, 6.28,
                                  6.28, 6.28, 6.28, 6.28, 6.28, 6.28, 6.28])
        self.max_qacc = np.array([1., 1., 1., 1., 40., 40.,
                                  10., 10., 10., 20., 40., 40., 40.,
                                  10., 10., 10., 20., 40., 40., 40.])

        self.home_qpos = np.deg2rad([45, -90, 45, 0, 0, 0,
                                     5, -45, -5, -90, 0, -15, 0,
                                     -5, 45, 5, 90, 0, 15, 0])
        self.update_params()
        # yapf: enable

    def update_params(self, file_path=DATA_DIR / "config.json") -> bool:
        if not file_path.exists() or file_path.stat().st_size == 0:
            self.logger.warning(
                f"{str(file_path)} does not exist, would be created automatically."
            )
            template_json_data = self._parse_to_dict()
            file_path.write_text(json.dumps(template_json_data,
                                            indent=4,
                                            ensure_ascii=False),
                                 encoding="utf-8")
            return False

        with open(file_path, "rb") as f:
            json_data = json.load(f)
            self._update_from_dict(json_data)

    def _update_from_dict(self, data: dict) -> bool:
        home_qpos = data.get("home_qpos")
        if home_qpos is not None:
            if len(home_qpos) != self.full_body_dofs:
                self.logger.error(f"home_qpos dimension mismatch!")
                return False
            else:
                self.home_qpos = np.array(home_qpos)
                self.logger.debug(f"Update home_qpos from json.")

        qpos_limits = data.get("qpos_limits")
        if qpos_limits is not None:
            if np.array(qpos_limits).shape != (self.full_body_dofs, 2):
                self.logger.error(f"qpos_limits dimension mismatch!")
                return False
            else:
                self.qpos_lb = np.array(qpos_limits)[:, 0]
                self.qpos_ub = np.array(qpos_limits)[:, 1]

                self.torso_lb = np.maximum(self.torso_lb,
                                           self.qpos_lb[self.torso_index])
                self.head_lb = np.maximum(self.head_lb,
                                          self.qpos_lb[self.head_index])
                self.left_arm_lb = np.maximum(self.left_arm_lb,
                                              self.qpos_lb[self.left_arm_index])
                self.right_arm_lb = np.maximum(
                    self.right_arm_lb, self.qpos_lb[self.right_arm_index])

                self.torso_ub = np.minimum(self.torso_ub,
                                           self.qpos_ub[self.torso_index])
                self.head_ub = np.minimum(self.head_ub,
                                          self.qpos_ub[self.head_index])
                self.left_arm_ub = np.minimum(self.left_arm_ub,
                                              self.qpos_ub[self.left_arm_index])
                self.right_arm_ub = np.minimum(
                    self.right_arm_ub, self.qpos_ub[self.right_arm_index])
                self.logger.debug(f"Update qpos_limits from json.")

        max_qvel = data.get("max_qvel")
        if max_qvel is not None:
            if len(max_qvel) != self.full_body_dofs:
                self.logger.error(f"max_qvel dimension mismatch!")
                return False
            else:
                self.max_qvel = np.clip(max_qvel, 0., 1.5)
                self.logger.debug(f"Update max_qvel from json.")

        max_qacc = data.get("max_qacc")
        if max_qacc is not None:
            if len(max_qacc) != self.full_body_dofs:
                self.logger.error(f"max_qacc dimension mismatch!")
                return False
            else:
                self.max_qacc = np.clip(max_qacc, 0., 15.)
                self.logger.debug(f"Update max_qacc from json.")

        default_qvel = data.get("default_qvel")
        if default_qvel is not None:
            if len(default_qvel) != self.full_body_dofs:
                self.logger.error(f"default_qvel dimension mismatch!")
                return False
            else:
                self.default_qvel = np.clip(
                    default_qvel, np.array([0.] * self.full_body_dofs),
                    self.max_qvel)
                self.logger.debug(f"Update default_qvel from json.")

    def _parse_to_dict(self) -> dict:
        data = {
            "home_qpos": self.home_qpos.tolist(),
            "qpos_limits": np.vstack((self.qpos_lb, self.qpos_ub)).T.tolist(),
            "max_qvel": self.max_qvel.tolist(),
            "max_qacc": self.max_qacc.tolist(),
            "default_qvel": self.default_qvel.tolist()
        }
        return data
