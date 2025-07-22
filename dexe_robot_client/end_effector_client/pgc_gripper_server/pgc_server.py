# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int32
import asyncio
from pgc_gripper import dh_modbus_gripper

# 配置参数
LEFT_GRIPPER_PORT = "/dev/ttyPGCGripper"  # 注意：左右夹爪应使用不同端口
RIGHT_GRIPPER_PORT = "/dev/ttyPGCGripper"
BAUDRATE = 115200
TIMEOUT = 10
LEFT_GRIPPER_FORCE = 30
LEFT_GRIPPER_SPEED = 50
RIGHT_GRIPPER_FORCE = 30
RIGHT_GRIPPER_SPEED = 50

async_loop = asyncio.new_event_loop()

def run_async_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

async_thread = threading.Thread(target=run_async_loop, args=(async_loop,), daemon=True)
async_thread.start()

class GripperManager:
    """单例模式管理串口连接"""

    _instances = {}
    _lock = asyncio.Lock()

    def __new__(cls, port, baudrate):
        if port not in cls._instances:
            instance = super().__new__(cls)
            instance.gripper = dh_modbus_gripper()
            try:
                instance.gripper.open(port, baudrate)
                print(f"Successfully opened {port}")
            except Exception as e:
                print(f"Failed to open {port}: {str(e)}")
                raise
            cls._instances[port] = instance
        return cls._instances[port]


class PGCNode(Node):
    def __init__(
        self,
        name: str,
        port: str,
        force: float = 50,
        speed: float = 50,
    ):
        super().__init__(name)
        self.name = name
        self.left_gripper_id = 0x01
        self.right_gripper_id = 0x02

        self.force = force
        self.speed = speed
        self.initstate = 0
        self.connected = False

        self.tasks = []

        # 初始化单例连接
        try:
            self.gripper = GripperManager(port, BAUDRATE).gripper
            self._init_gripper()
        except Exception as e:
            self.get_logger().error(f"Connection failed: {str(e)}")
            return

        # 初始化发布/订阅
        self._init_publishers()
        self._init_subscribers()
        # self.create_timer(1.0, self.publish_gripper_states)

    def _init_gripper(self, timeout=TIMEOUT):
        """设备专属初始化"""

        async def _async_init():
            try:
                # 设备初始化
                await self.gripper.Initialization(self.left_gripper_id)
                await self.gripper.Initialization(self.right_gripper_id)

                # 参数设置
                await self.gripper.SetTargetForce(self.force, self.left_gripper_id)
                await self.gripper.SetTargetSpeed(self.speed, self.left_gripper_id)
                await self.gripper.SetTargetForce(self.force, self.right_gripper_id)
                await self.gripper.SetTargetSpeed(self.speed, self.right_gripper_id)

                # 状态验证
                start_time = time.time()
                while (state := await self.gripper.GetInitState(self.left_gripper_id)) != 1:
                    if time.time() - start_time > timeout:
                        raise TimeoutError("Left gripper initialization timeout")
                    await asyncio.sleep(0.2)

                while (state := await self.gripper.GetInitState(self.right_gripper_id)) != 1:
                    if time.time() - start_time > timeout:
                        raise TimeoutError("Right gripper initialization timeout")
                    await asyncio.sleep(0.2)

                self.connected = True
                self.get_logger().info("Both grippers initialized")
            except Exception as e:
                self.get_logger().error(f"Init failed: {str(e)}")
                self.connected = False

        asyncio.run(_async_init())

    def _init_publishers(self):
        """初始化发布器"""
        self.left_grip_state_publisher = self.create_publisher(
            Int32, f"/left_pgc_node/grip_state", 10
        )
        self.left_current_position_publisher = self.create_publisher(
            Float64, f"/left_pgc_node/current_position", 10
        )

        self.right_grip_state_publisher = self.create_publisher(
            Int32, f"/right_pgc_node/grip_state", 10
        )
        self.right_current_position_publisher = self.create_publisher(
            Float64, f"/right_pgc_node/current_position", 10
        )

    def _init_subscribers(self):
        """初始化订阅器"""
        self.create_subscription(
            Float64MultiArray,
            f"/set_left_pgc_position",
            self._left_position_callback,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            f"/set_right_pgc_position",
            self._right_position_callback,
            10,
        )

        self.create_subscription(
            Float64, f"/set_left_pgc_force", self._left_force_callback, 10
        )

        self.create_subscription(
            Float64, f"/set_right_pgc_force", self._right_force_callback, 10
        )

        self.create_subscription(
            Float64, f"/set_left_pgc_speed", self._left_speed_callback, 10
        )

        self.create_subscription(
            Float64, f"/set_right_pgc_speed", self._right_speed_callback, 10
        )

    def _left_position_callback(self, msg):
        """左夹爪位置指令回调"""
        if len(msg.data) < 1:
            return

        angle = msg.data[0]
        if 0 <= angle <= 1000:
            asyncio.run(
                    self.gripper.SetTargetPosition(int(angle), self.left_gripper_id)
            )
        else:
            self.get_logger().warn(f"Invalid left gripper target position: {angle}")

    def _right_position_callback(self, msg):
        """右夹爪位置指令回调"""
        if len(msg.data) < 1:
            return

        angle = msg.data[0]
        if 0 <= angle <= 1000:
            asyncio.run(
                    self.gripper.SetTargetPosition(int(angle), self.right_gripper_id)
                )
        else:
            self.get_logger().warn(f"Invalid right gripper target position: {angle}")

    def _left_force_callback(self, msg):
        """左夹爪力指令回调"""
        if len(msg.data) < 1:
            return
        asyncio.run(
                self.gripper.SetTargetForce(int(msg.data[0]), self.left_gripper_id)
            )

    def _right_force_callback(self, msg):
        """右夹爪力指令回调"""
        if len(msg.data) < 1:
            return
        asyncio.run(
                self.gripper.SetTargetForce(int(msg.data[0]), self.right_gripper_id)
            )

    def _left_speed_callback(self, msg):
        """左夹爪速度指令回调"""
        if len(msg.data) < 1:
            return
        asyncio.run(
                self.gripper.SetTargetSpeed(int(msg.data[0]), self.left_gripper_id)
            )

    def _right_speed_callback(self, msg):
        """右夹爪速度指令回调"""
        if len(msg.data) < 1:
            return
        asyncio.run(
                self.gripper.SetTargetSpeed(int(msg.data[0]), self.right_gripper_id)
            )

    def publish_gripper_states(self):
        """状态发布"""
        if not self.connected:
            return

        async def _publish():
            try:
                left_state = await self.gripper.GetGripState(self.left_gripper_id)
                self.left_grip_state_publisher.publish(Int32(data=int(left_state)))

                left_position = await self.gripper.GetCurrentPosition(self.left_gripper_id)
                print("left_position: ", left_position)
                self.left_current_position_publisher.publish(Float64(data=float(left_position)))

                right_state = await self.gripper.GetGripState(self.right_gripper_id)
                self.right_grip_state_publisher.publish(Int32(data=int(right_state)))

                right_position = await self.gripper.GetCurrentPosition(self.right_gripper_id)
                print("right_position: ", right_position)
                self.right_current_position_publisher.publish(Float64(data=float(right_position)))
            except Exception as e:
                self.get_logger().warn(f"Publish failed: {str(e)}")

        # task = asyncio.create_task(_publish())
        future = asyncio.run_coroutine_threadsafe(_publish(), async_loop)        
        self.tasks.append(future)

    def destroy_node(self):
        """清理任务"""
        for task in self.tasks:
            task.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    # 创建 PGCNode 实例
    gripper_node = PGCNode(
        name="pgc_node",
        port=LEFT_GRIPPER_PORT,
        force=LEFT_GRIPPER_FORCE,
        speed=LEFT_GRIPPER_SPEED,
    )

    try:
        # 添加节点到执行器并运行
        executor.add_node(gripper_node)
        executor.spin()
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 以安全退出
        gripper_node.get_logger().info("Shutting down gracefully...")
    finally:
        # 确保资源释放
        executor.shutdown()
        gripper_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
