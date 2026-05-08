# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import rclpy
from dexe_rp.utils.type import ArmType, W1Version, EndEffectorType
from dexe_robot_client.sim_server import ViserServer


def main():
    rclpy.init()

    arm_type = ArmType.ANTHROPOMORPHIC  # ArmType.INDUSTRIAL
    w1_version = W1Version.V024  # W1Version.V023, W1Version.V024i, W1Version.V023i
    end_effector_type = EndEffectorType.BrainCo_Revo1_R  # None, EndEffectorType.DH_PGC_140_50

    viser_server = ViserServer(arm_type=arm_type,
                               w1_version=w1_version,
                               end_effector_type=end_effector_type)

    try:
        viser_server.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
