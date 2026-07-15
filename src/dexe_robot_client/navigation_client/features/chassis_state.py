"""底盘状态：位姿 / 电量 / 障碍物订阅缓存 + 连接检测。

时间源统一 time.monotonic()（单调钟，不受系统时间/仿真时间影响）。
连接超时会同时清空位姿缓存——断连后 get_position() 返回 (False, None)，
不再返回过期位姿。
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool


class ChassisStateFeature:

    # 电量诊断源（底盘诊断结构变化时改这里）
    BATTERY_DIAG_NAME = "/Devices/System/PowerBoard/collector_node: PowerBoard"
    BATTERY_DIAG_KEY = "battery_percentage"

    def __init__(self, node, cfg, connection_timeout: float = 1.0):
        self._node = node
        self._connection_timeout = connection_timeout
        self._last_msg_time = None
        self._last_pose = None            # (position[3], quaternion[4])
        self._disconnected_logged = False
        self._battery_percentage = -1.0
        self._has_obstacle = False

        node.create_subscription(
            PoseStamped, cfg.chassis_pose_topic, self._on_pose, 10)
        if cfg.battery_topic:
            node.create_subscription(
                DiagnosticArray, cfg.battery_topic, self._on_battery, 10)
        if cfg.obstacle_topic:
            # 仅真实模式订阅（仿真端当前不提供该话题）
            node.create_subscription(
                Bool, cfg.obstacle_topic, self._on_obstacle, 10)
        node.create_timer(0.1, self._check_connection)

    # ------------------------------------------------ 回调
    def _on_pose(self, msg):
        self._node.get_logger().info(
            "[底盘位姿反馈已连接] 已收到首条底盘位姿消息。", once=True)
        if self._disconnected_logged:
            self._node.get_logger().info("[底盘位姿反馈恢复] 已重新收到底盘位姿消息。")
            self._disconnected_logged = False
        self._last_pose = (
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w],
        )
        self._last_msg_time = time.monotonic()

    def _on_battery(self, msg):
        for status in msg.status:
            if status.name == self.BATTERY_DIAG_NAME:
                for key_value in status.values:
                    if key_value.key == self.BATTERY_DIAG_KEY:
                        self._battery_percentage = float(key_value.value)
                        break

    def _on_obstacle(self, msg):
        self._has_obstacle = msg.data

    def _check_connection(self):
        if self._last_msg_time is None:
            return
        if time.monotonic() - self._last_msg_time > self._connection_timeout:
            # 断连：位姿缓存一并清空，杜绝"返回过期位姿"
            self._last_msg_time = None
            self._last_pose = None
            self._disconnected_logged = True
            self._node.get_logger().warn(
                f"[底盘位姿反馈中断] 超过 {self._connection_timeout:.1f}s "
                "未收到底盘位姿，位姿查询将返回失败。")

    # ------------------------------------------------ 查询
    def connected(self) -> bool:
        return self._last_msg_time is not None

    def get_position(self):
        """返回 (success, np.ndarray([x, y, yaw_deg])) 或 (False, None)。"""
        pose = self._last_pose
        if pose is None:
            self._node.get_logger().error("[底盘位姿不可用] 无有效底盘位姿反馈。")
            return False, None
        pos, quat = pose
        yaw = R.from_quat(np.array(quat)).as_euler('xyz', degrees=True)[2]
        return True, np.array([pos[0], pos[1], yaw])

    def battery(self):
        if self._battery_percentage < 0:
            return False, -1.0
        return True, self._battery_percentage

    def obstacle(self) -> bool:
        return self._has_obstacle
