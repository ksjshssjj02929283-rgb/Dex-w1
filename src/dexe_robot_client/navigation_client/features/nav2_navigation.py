"""Nav2 导航功能：ActionTaskClient(NavigateToPose) + goal 构造 + 旧状态取值映射。"""

from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from ..common import ActionTaskClient, TaskState, parse_pose3

# 对外保持旧取值：IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELLED
_LEGACY = {
    TaskState.IDLE: "IDLE",
    TaskState.WAITING_SERVER: "NAVIGATING",
    TaskState.SENDING: "NAVIGATING",
    TaskState.ACCEPTED: "NAVIGATING",
    TaskState.EXECUTING: "NAVIGATING",
    TaskState.CANCELING: "NAVIGATING",
    TaskState.SUCCEEDED: "SUCCEEDED",
    TaskState.CANCELED: "CANCELLED",
    TaskState.ABORTED: "FAILED",
    TaskState.REJECTED: "FAILED",
    TaskState.FAILED_TO_SEND: "FAILED",
    TaskState.LOST: "FAILED",
}


def _feedback_summary(feedback) -> str:
    nav_time = (feedback.navigation_time.sec +
                feedback.navigation_time.nanosec * 1e-9)
    return (f"剩余距离={feedback.distance_remaining:.3f}m，"
            f"已导航={nav_time:.1f}s，"
            f"恢复行为次数={feedback.number_of_recoveries}")


class Nav2NavigationFeature:

    def __init__(self, node, cfg):
        self._node = node
        self._task = ActionTaskClient(node,
                                      NavigateToPose,
                                      cfg.nav2_action,
                                      label="导航",
                                      wait_server_timeout=1.0,
                                      feedback_formatter=_feedback_summary)

    def send(self, position, *, trigger_ts: float = None) -> bool:
        vals = parse_pose3(position)
        if vals is None:
            self._node.get_logger().error(
                f"[导航目标非法] 期望 [x, y, yaw_deg] 三个有限数值，收到：{position}")
            return False
        x, y, yaw_deg = vals

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        quat = R.from_euler('xyz', [0.0, 0.0, yaw_deg], degrees=True).as_quat()
        goal.pose.pose.orientation.x = quat[0]
        goal.pose.pose.orientation.y = quat[1]
        goal.pose.pose.orientation.z = quat[2]
        goal.pose.pose.orientation.w = quat[3]

        sent = self._task.send(goal, trigger_ts=trigger_ts)
        if sent:
            self._node.get_logger().info(
                f"[导航目标] ({x:.3f}, {y:.3f}, {yaw_deg:.1f}°)，坐标系=map。")
        return sent

    def state(self) -> str:
        return _LEGACY[self._task.state]

    def has_cancelable_goal(self) -> bool:
        return self._task.is_active()

    def cancel(self) -> bool:
        return self._task.cancel()
