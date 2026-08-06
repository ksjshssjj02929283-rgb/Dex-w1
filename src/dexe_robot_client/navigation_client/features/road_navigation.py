"""路网 / 多路点导航：ActionTaskClient(NavigateThroughPoses)。

⚠️ 单位与 Nav2 导航（度）不同：waypoints 的 yaw 用**弧度**，与 Nav2
NavigateThroughPoses 原生接口一致（W1Manager 侧沿用此约定，未在本次改动）。
"""

import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses

from ..common import ActionTaskClient, TaskState

_LEGACY = {
    TaskState.IDLE: "IDLE",
    TaskState.WAITING_SERVER: "WAITING_SERVER",
    TaskState.SENDING: "SENDING",
    TaskState.ACCEPTED: "ACCEPTED",
    TaskState.EXECUTING: "EXECUTING",
    TaskState.CANCELING: "CANCELING",
    TaskState.SUCCEEDED: "SUCCEEDED",
    TaskState.CANCELED: "CANCELED",
    TaskState.ABORTED: "ABORTED",
    TaskState.REJECTED: "REJECTED",
    TaskState.FAILED_TO_SEND: "FAILED_TO_SEND",
    TaskState.LOST: "UNKNOWN",
}


def _feedback_summary(feedback) -> str:
    return (f"剩余路点={feedback.number_of_poses_remaining}，"
            f"剩余距离={feedback.distance_remaining:.3f}m")


def _make_pose(node, x: float, y: float, yaw_rad: float,
               frame_id: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    half_yaw = float(yaw_rad) * 0.5
    pose.pose.orientation.z = math.sin(half_yaw)
    pose.pose.orientation.w = math.cos(half_yaw)
    return pose


class RoadNavigationFeature:

    def __init__(self, node, cfg):
        self._node = node
        self._task = ActionTaskClient(node,
                                      NavigateThroughPoses,
                                      cfg.road_nav_action,
                                      label="路网导航",
                                      wait_server_timeout=10.0,
                                      feedback_formatter=_feedback_summary)

    def send(self,
             waypoints: list,
             frame_id: str = "map",
             timeout_sec: float = 10.0,
             behavior_tree: str = "",
             *,
             trigger_ts: float = None) -> bool:
        if not waypoints:
            self._node.get_logger().error("[路网导航目标非法] waypoints 为空。")
            return False
        try:
            goal = NavigateThroughPoses.Goal()
            goal.poses = [
                _make_pose(self._node, x, y, yaw, frame_id)
                for x, y, yaw in waypoints
            ]
            goal.behavior_tree = behavior_tree
        except (TypeError, ValueError) as e:
            self._node.get_logger().error(f"[路网导航目标非法] {e}")
            return False
        return self._task.send(goal,
                               wait_server_timeout=timeout_sec,
                               trigger_ts=trigger_ts)

    def status(self) -> str:
        return _LEGACY[self._task.state]

    def result(self):
        ok, msg = self._task.result()
        return ok, (msg or self.status())

    def has_cancelable_goal(self) -> bool:
        return self._task.is_active()

    def cancel(self) -> bool:
        return self._task.cancel()
