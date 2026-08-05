"""路径导航：ActionTaskClient(NavigateThroughGraphPoints)。

按 `nav_point_names` 串接路径，由服务端调度器解析路点并逐段执行。
客户端不校验点位数量、不拦截并发 goal，是否接受完全由服务端裁决。
"""

from nav_interfaces.action import NavigateThroughGraphPoints

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
    return (f"状态={feedback.current_status}，"
            f"点={feedback.current_point_index}/{feedback.total_points}")


class GraphPointsNavigationFeature:

    def __init__(self, node, cfg):
        self._node = node
        self._task = ActionTaskClient(node,
                                      NavigateThroughGraphPoints,
                                      cfg.graph_points_nav_action,
                                      label="路径导航",
                                      wait_server_timeout=10.0,
                                      feedback_formatter=_feedback_summary)

    def send(self,
             nav_point_names: list,
             nav_point_ids: list = None,
             pose=None,
             target_yaw: float = None,
             timeout_sec: float = 10.0,
             *,
             trigger_ts: float = None) -> bool:
        try:
            goal = NavigateThroughGraphPoints.Goal()
            goal.nav_point_names = [str(name) for name in nav_point_names]
            if nav_point_ids is not None:
                goal.nav_point_ids = [str(pid) for pid in nav_point_ids]
            if pose is not None:
                goal.pose = pose
            if target_yaw is not None:
                goal.target_yaw = float(target_yaw)
        except (TypeError, ValueError) as e:
            self._node.get_logger().error(f"[路径导航目标非法] {e}")
            return False
        return self._task.send(goal,
                               wait_server_timeout=timeout_sec,
                               trigger_ts=trigger_ts)

    def status(self) -> str:
        return _LEGACY[self._task.state]

    def result(self):
        return self._task.result()

    def has_cancelable_goal(self) -> bool:
        return self._task.is_active()

    def cancel(self) -> bool:
        return self._task.cancel()
