'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from dexe_robot_client.common import StatusCode, StatusContext

if __name__ == "__main__":
    print(f"{StatusContext.get(StatusCode.SUCCESS)}")
    print(f"{StatusContext.get(StatusCode.FAIL)}")

    print(f"{StatusContext.get(StatusCode.INPUT_INVALID)}")
    print(f"{StatusContext.get(StatusCode.DIM_MISMATCH)}")
    print(f"{StatusContext.get(StatusCode.NAME_MISMATCH)}")
    print(f"{StatusContext.get(StatusCode.INPUT_EMPTY)}")

    print(f"{StatusContext.get(StatusCode.ROBOT_STATUS_INVALID)}")
    print(f"{StatusContext.get(StatusCode.ROBOT_NOT_CONNECT)}")
    print(f"{StatusContext.get(StatusCode.ROBOT_NOT_ENABLE)}")
    print(f"{StatusContext.get(StatusCode.ROBOT_MOVE_FAIL)}")

    print(f"{StatusContext.get(StatusCode.EE_NOT_CONNECT)}")
    print(f"{StatusContext.get(StatusCode.EE_MOVE_FAIL)}")

    print(f"{StatusContext.get(StatusCode.CHASSIS_NOT_CONNECT)}")

    print(f"{StatusContext.get(StatusCode.IK_FAIL)}")
    print(f"{StatusContext.get(StatusCode.FK_FAIL)}")
    print(f"{StatusContext.get(StatusCode.EXCEED_LIMIT)}")
    print(f"{StatusContext.get(StatusCode.PLAN_PATH_FAIL)}")
    print(f"{StatusContext.get(StatusCode.PLAN_TRAJ_FAIL)}")
    print(f"{StatusContext.get(StatusCode.COLLISION_FAIL)}")

    print(f"{StatusContext.get(StatusCode.NOT_AT_START)}")
    print(f"{StatusContext.get(StatusCode.GOAL_NOT_REACH)}")

    print(f"{StatusContext.get(StatusCode.STOP_MOVE)}")
    print(f"{StatusContext.get(StatusCode.TIME_OUT)}")

    print(f"{StatusContext.get(None)}")
