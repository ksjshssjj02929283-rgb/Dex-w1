"""迦智底盘精调：将相对直行/旋转转换成任务接口。"""

from ...common import TaskState, parse_pose3
from .http import CloudiaApiError


class CloudiaPreciseControlFeature:

    def __init__(self, node, task):
        self._node = node
        self._task = task

    def send(self, position, *, trigger_ts=None):
        values = parse_pose3(position)
        if values is None:
            self._node.get_logger().error(
                f"[迦智底盘精调] 目标应为有限数值 [x, 0, yaw]，收到：{position}")
            return False
        x, y, yaw_deg = values
        if abs(y) > 1e-9 or (abs(x) > 1e-9 and abs(yaw_deg) > 1e-9):
            self._node.get_logger().error("[迦智底盘精调] x 与 yaw 不能同时非零，且 y 必须为 0。")
            return False
        try:
            if abs(x) > 1e-9:
                return self._task.start("move",
                                        distance=x,
                                        trigger_ts=trigger_ts)
            if abs(yaw_deg) > 1e-9:
                return self._task.start("rotate",
                                        angle=yaw_deg,
                                        trigger_ts=trigger_ts)
        except CloudiaApiError as exc:
            self._node.get_logger().error(f"[迦智底盘精调] {exc}")
        return False

    def status(self, consume=True):
        del consume
        state = self._task.state
        if state in {
                TaskState.SENDING, TaskState.ACCEPTED, TaskState.EXECUTING
        }:
            return "CONTROL"
        if state == TaskState.CANCELING:
            return "CANCELLING"
        if state == TaskState.CANCELED:
            return "CANCELLED"
        if state == TaskState.SUCCEEDED:
            return "SUCCEEDED"
        if state in {
                TaskState.ABORTED, TaskState.REJECTED, TaskState.FAILED_TO_SEND,
                TaskState.LOST
        }:
            return "FAILED"
        return "IDLE"

    def refresh(self):
        self._task.refresh()
        return True

    def has_cancelable_goal(self):
        return self._task.is_active()

    def cancel(self):
        return self._task.cancel()
