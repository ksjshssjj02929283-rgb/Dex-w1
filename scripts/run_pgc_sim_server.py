# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import rclpy
from dexe_robot_client.sim_server import PGCGrippersSimServer


def main():
    rclpy.init()
    node = PGCGrippersSimServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("Shutting down gracefully...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()