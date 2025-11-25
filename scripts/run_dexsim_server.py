# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import rclpy
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.sim_server import DexsimServer


def main():
    rclpy.init()
    dexsim_server = DexsimServer(arm_type=W1ArmType.ANTHROPOMORPHIC,
                                 w1_version=W1Version.V2,
                                 end_effector_type=EndEffectorType.BRAINCO_HAND)

    # dexsim_server = DexsimServer(arm_type = W1ArmType.INDUSTRIAL,
    #                              w1_version = W1Version.V2,
    #                              end_effector_type = EndEffectorType.PGC_GRIPPER)

    # Used to generate assebled urdf without end effector
    # dexsim_server = DexsimServer(arm_type = W1ArmType.ANTHROPOMORPHIC,
    #                             w1_version = W1Version.V2,
    #                             end_effector_type = None)

    try:
        dexsim_server.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
