# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int32
from pgc_gripper import dh_modbus_gripper

# 配置参数
GRIPPER_PORT = "/dev/ttyUSB_485"
BAUDRATE = 115200
TIMEOUT = 10
GRIPPER_FORCE = 30
GRIPPER_SPEED = 100

class PGCNode(Node):
    def __init__(
        self,
        name: str,
        port: str,
        baud_rate: int, 
        force: float = 50,
        speed: float = 50,
    ):
        super().__init__(name)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.left_gripper_id = 0x01
        self.right_gripper_id = 0x02

        self.force = force
        self.speed = speed
        self.initstate = 0
        self.connected = False

        self.gripper = dh_modbus_gripper()
        try:
            self.gripper.open(port, baud_rate)
        except Exception as e:
            self.get_logger().error(f"Connection failed: {str(e)}")
            return

        try:            
            self._init_gripper()
        except Exception as e:
            self.get_logger().error(f"Init failed: {str(e)}")
            return

        # 初始化发布/订阅
        self._init_publishers()
        self._init_subscribers()
        self.create_timer(0.1, self._publish_states)

    def _init_gripper(self, timeout=TIMEOUT):
        """设备专属初始化"""
        try:
            # 设备初始化
            self.gripper.Initialization(self.left_gripper_id)
            # 状态验证
            init_state = 0
            start_time = time.time()
            while init_state != 1:
                init_state = self.gripper.GetInitState(self.left_gripper_id)
                if time.time() - start_time > timeout:
                    raise TimeoutError("Left gripper initialization timeout")
                time.sleep(0.2)
            # 参数设置
            self.gripper.SetTargetForce(self.force, self.left_gripper_id)
            self.gripper.SetTargetSpeed(self.speed, self.left_gripper_id)
            self.get_logger().info(f"Left gripper initialized")

            self.gripper.Initialization(self.right_gripper_id)
            init_state = 0
            while init_state != 1:
                init_state = self.gripper.GetInitState(self.right_gripper_id)
                if time.time() - start_time > timeout:
                    raise TimeoutError("Right gripper initialization timeout")
                time.sleep(0.2)
            self.gripper.SetTargetForce(self.force, self.right_gripper_id)
            self.gripper.SetTargetSpeed(self.speed, self.right_gripper_id)
            self.get_logger().info(f"Right gripper initialized")

            self.connected = True
        except Exception as e:
            self.gripper.close()
            self.connected = False
            raise 

    def _init_publishers(self):
        """初始化发布器"""
        self.left_grip_state_publisher = self.create_publisher(
            Int32, f"/left_pgc_node/grip_state", 10
        )
        self.left_qpos_publisher = self.create_publisher(
            Float64, f"/left_pgc_node/current_position", 10
        )

        self.right_grip_state_publisher = self.create_publisher(
            Int32, f"/right_pgc_node/grip_state", 10
        )
        self.right_qpos_publisher = self.create_publisher(
            Float64, f"/right_pgc_node/current_position", 10
        )

    def _init_subscribers(self):
        """初始化订阅器"""
        self.create_subscription(
            Float64MultiArray,
            f"/set_left_pgc_position",
            self._left_qpos_callback,
            10,
        )
        self.create_subscription(
            Float64, f"/set_left_pgc_force", self._left_force_callback, 10
        )
        self.create_subscription(
            Float64, f"/set_left_pgc_speed", self._left_speed_callback, 10
        )

        self.create_subscription(
            Float64MultiArray,
            f"/set_right_pgc_position",
            self._right_qpos_callback,
            10,
        )
        self.create_subscription(
            Float64, f"/set_right_pgc_force", self._right_force_callback, 10
        )
        self.create_subscription(
            Float64, f"/set_right_pgc_speed", self._right_speed_callback, 10
        )

    def _left_qpos_callback(self, msg):
        """左夹爪位置指令回调"""
        angle = msg.data[0]
        if 0. <= angle <= 1000.:
            self.gripper.SetTargetPosition(int(angle), self.left_gripper_id)
        else:
            self.get_logger().warning(f"Invalid left gripper target position: {angle}")

    def _right_qpos_callback(self, msg):
        """右夹爪位置指令回调"""
        angle = msg.data[0]
        if 0. <= angle <= 1000.:
            self.gripper.SetTargetPosition(int(angle), self.right_gripper_id)
        else:
            self.get_logger().warning(f"Invalid right gripper target position: {angle}")

    def _left_force_callback(self, msg):
        """左夹爪力指令回调"""
        force = msg.data
        if 20 <= force <= 100:
            self.gripper.SetTargetForce(int(force), self.left_gripper_id)
        else:
            self.get_logger().warning(f"Invalid left gripper target force: {force}")

    def _right_force_callback(self, msg):
        """右夹爪力指令回调"""
        force = msg.data
        if 20 <= force <= 100:
            self.gripper.SetTargetForce(int(force), self.right_gripper_id)
        else:
            self.get_logger().warning(f"Invalid right gripper target force: {force}")

    def _left_speed_callback(self, msg):
        """左夹爪速度指令回调"""
        speed = msg.data
        if 0 <= speed <= 100:
            self.gripper.SetTargetSpeed(int(speed), self.left_gripper_id)
        else:
            self.get_logger().warning(f"Invalid left gripper target speed: {speed}")

    def _right_speed_callback(self, msg):
        """右夹爪速度指令回调"""
        speed = msg.data
        if 0 <= speed <= 100:
            self.gripper.SetTargetSpeed(int(speed), self.right_gripper_id)
        else:
            self.get_logger().warning(f"Invalid right gripper target speed: {speed}")

    def _publish_states(self):
        """状态发布"""
        if not self.connected:
            return
        try:
            left_grip_state = self.gripper.GetGripState(self.left_gripper_id)
            left_qpos = self.gripper.GetCurrentPosition(self.left_gripper_id)
            self.left_grip_state_publisher.publish(Int32(data=int(left_grip_state)))
            self.left_qpos_publisher.publish(Float64(data=float(left_qpos)))
            self.get_logger().debug(f"left grip state: {left_grip_state}")
            self.get_logger().debug(f"left qpos: {left_qpos}")

            right_grip_state = self.gripper.GetGripState(self.right_gripper_id)
            right_qpos = self.gripper.GetCurrentPosition(self.right_gripper_id)
            self.right_grip_state_publisher.publish(Int32(data=int(right_grip_state)))
            self.right_qpos_publisher.publish(Float64(data=float(right_qpos)))
            self.get_logger().debug(f"right grip state: {right_grip_state}")
            self.get_logger().debug(f"right qpos: {right_qpos}")
        except Exception as e:
            self.get_logger().warning(f"Publish failed: {str(e)}")

    def destroy_node(self):
        """清理任务"""
        self.gripper.close()
        super().destroy_node()

def main():
    rclpy.init()
    try:
        node = PGCNode(
            name="pgc_server_node",
            port=GRIPPER_PORT,
            baud_rate=BAUDRATE,
            force=GRIPPER_FORCE,
            speed=GRIPPER_SPEED,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
