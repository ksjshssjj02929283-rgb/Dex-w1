# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import numpy as np
from dexe_robot_client.common import W1Version, W1ArmType, get_logger
from dexe_robot_client.w1 import W1Params, SRSKinematics


def check_kine(is_left: bool = True,
               arm_type: W1ArmType = W1ArmType.ANTHROPOMORPHIC):
    w1_params = W1Params(arm_type, W1Version.V2, None)

    if is_left:
        kine_params = w1_params._left_arm_kine_params
        get_logger().info(f"\n\nCheck kine: left {arm_type.name}")
    else:
        kine_params = w1_params._right_arm_kine_params
        get_logger().info(f"\n\nCheck kine: right {arm_type.name}")

    srs_kine = SRSKinematics(kine_params)

    lower_bound = kine_params.qpos_limits[:, 0]
    upper_bound = kine_params.qpos_limits[:, 1]
    qpos = np.random.uniform(lower_bound, upper_bound)

    # 测试运动学求解耗时
    n = 1000
    t0 = time.time()
    for i in range(n):
        res_fk, xpos = srs_kine.get_fk(qpos)
    get_logger().info(f"fk time consuming: {(time.time() - t0)/n*1000} ms")

    t0 = time.time()
    for i in range(n):
        srs_kine.get_nearst_ik(xpos, qpos)
    get_logger().info(f"ik time consuming: {(time.time() - t0)/n*1000} ms")

    # 检查运动学正确性
    for i in range(n):
        qpos = np.random.uniform(lower_bound, upper_bound)
        res, xpos = srs_kine.get_fk(qpos)
        if not res:
            get_logger().error(f"fk failed: {i}")
            break
        res, qpos_ = srs_kine.get_nearst_ik(xpos, qpos)
        if not res:
            get_logger().error(f"ik failed: {i}")
            break
        if not np.allclose(qpos_, qpos):
            get_logger().error(f"ref arm angle failed: {i}")
            break
        res, xpos_ = srs_kine.get_fk(qpos_)
        if not res:
            get_logger().error(f"fk failed: {i}")
            break
        if np.sum(abs(xpos_ - xpos)) >= 1e-6:
            get_logger().error(f"xpos error failed: {i}")
            break
    get_logger().info(f"check kine successed")


if __name__ == "__main__":
    check_kine(is_left=True, arm_type=W1ArmType.ANTHROPOMORPHIC)
    check_kine(is_left=False, arm_type=W1ArmType.ANTHROPOMORPHIC)
    check_kine(is_left=True, arm_type=W1ArmType.INDUSTRIAL)
    check_kine(is_left=False, arm_type=W1ArmType.INDUSTRIAL)
