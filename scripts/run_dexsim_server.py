# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import rclpy
from rlia.kit.drive_controllers.w1_drive_controller import W1Version, W1ArmType
from dexe_robot_client.sim_server import DexsimServer

def main():
    rclpy.init()
    dexsim_server = DexsimServer(arm_type = W1ArmType.ANTHROPOMORPHIC)
    
    try:
        dexsim_server.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()