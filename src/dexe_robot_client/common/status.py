# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from enum import Enum


# yapf: disable
class StatusCode(Enum):
    SUCCESS = 1

    FAIL = -100

    INPUT_INVALID = -201
    DIM_MISMATCH = -202
    NAME_MISMATCH = -204

    ROBOT_STATUS_ERROR = -300
    ROBOT_NOT_CONNECT = -301
    ROBOT_NOT_ENABLE = -302

    EE_NOT_CONNECT = -400

    CHASSIS_NOT_CONNECT = -500

    IK_FAIL = -601
    FK_FAIL = -602
    EXCEED_LIMIT = -603
    PLAN_PATH_FAIL = -604
    PLAN_TRAJ_FAIL = -605
    COLLISION_FAIL = -606

    NOT_AT_START = -700
    GOAL_NOT_REACH = -701

    STOP_MOVE = -800


class StatusContext:
    STATUS_MESSAGES = {
        StatusCode.SUCCESS: "Success",

        StatusCode.FAIL: "Failed",

        StatusCode.INPUT_INVALID: "Input invalid",
        StatusCode.DIM_MISMATCH: "Dimension mismatch",
        StatusCode.NAME_MISMATCH: "Name mismatch",

        StatusCode.ROBOT_STATUS_ERROR: "Robot status mismatch",
        StatusCode.ROBOT_NOT_CONNECT: "Robot not connect",
        StatusCode.ROBOT_NOT_ENABLE: "Robot not enable",

        StatusCode.EE_NOT_CONNECT: "Endeffector not connect",

        StatusCode.CHASSIS_NOT_CONNECT: "Chassis not connect",

        StatusCode.IK_FAIL: "IK solve failed",
        StatusCode.FK_FAIL: "FK solve failed",
        StatusCode.EXCEED_LIMIT: "Exceed limit",
        StatusCode.PLAN_PATH_FAIL: "Plan path failed",
        StatusCode.PLAN_TRAJ_FAIL: "Plan trajectory failed",
        StatusCode.COLLISION_FAIL: "Found collision",

        StatusCode.NOT_AT_START: "Current qpos not at start qpos of trajectory",
        StatusCode.GOAL_NOT_REACH: "Final qpos not reach the final qpos of trajectory",

        StatusCode.STOP_MOVE: "Stop move by user"
    }

    @staticmethod
    def get(status_code):
        return StatusContext.STATUS_MESSAGES.get(status_code, "Unknown status code")
# yapf: enable
