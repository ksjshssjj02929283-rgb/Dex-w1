"""底盘精控功能：ActionTaskClient(PreciseControl)。

协议：`control/precise_control` action（`nav_interfaces/action/PreciseControl`，
server 见 mobile_perception/src/mobile_control_action_node.cpp）。
Goal.target_pose 在 base_link 系给相对位移/转角，server 内部会 TF 转换到 map
执行；x 与 yaw 不能同时非零。Feedback.current_status 是 server 定义的状态字符串
（IDLE/CONTROL/CANCELLING/SUCCEEDED），本 feature 直接透传，不做二次映射
（终态由 action result 二次确认，与 feedback 独立）。
"""

from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from nav_interfaces.action import PreciseControl

from ..common import ActionTaskClient, TaskState, parse_pose3

# result 未产出前，用 feedback 透传的 server 状态字符串作为查询结果；
# result 产出后按 action 终态覆盖为下列取值（与旧版 get_control_status 语义一致）。
_TERMINAL_LEGACY = {
    TaskState.SUCCEEDED: "SUCCEEDED",
    TaskState.CANCELED: "CANCELLED",
    TaskState.ABORTED: "FAILED",
    TaskState.REJECTED: "FAILED",
    TaskState.FAILED_TO_SEND: "FAILED",
    TaskState.LOST: "FAILED",
}


class PreciseControlFeature:

    def __init__(self, node, cfg):
        self._node = node
        self._task = ActionTaskClient(node,
                                      PreciseControl,
                                      cfg.precise_control_action,
                                      label="底盘精控",
                                      wait_server_timeout=1.0)
        self._last_feedback_status = "IDLE"

    def send(self, position, *, trigger_ts: float = None) -> bool:
        vals = parse_pose3(position)
        if vals is None:
            self._node.get_logger().error(
                f"[底盘直控目标非法] 期望 [x, 0, yaw_deg] 三个有限数值，收到：{position}")
            return False
        x, _, yaw_deg = vals
        if x != 0.0 and yaw_deg != 0.0:
            self._node.get_logger().error("[底盘直控目标非法] x 和 yaw 不能同时为非零值，"
                                          f"当前目标=({x:.3f}, {yaw_deg:.1f}°)。")
            return False

        goal = PreciseControl.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.pose.position.x = x
        quat = R.from_euler('xyz', [0.0, 0.0, yaw_deg], degrees=True).as_quat()
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self._last_feedback_status = "IDLE"
        return self._task.send(goal, trigger_ts=trigger_ts)

    def status(self, consume: bool = True) -> str:
        """IDLE/CONTROL/SUCCEEDED/CANCELLING/CANCELLED/FAILED/UNKNOWN。

        Args:
            consume: 兼容旧接口参数，当前不消费终态（历史遗留签名）。
        """
        del consume
        state = self._task.state
        if state in _TERMINAL_LEGACY:
            return _TERMINAL_LEGACY[state]
        feedback, _ = self._task.last_feedback()
        if feedback is not None and feedback.current_status:
            self._last_feedback_status = feedback.current_status
        return self._last_feedback_status

    def refresh(self) -> bool:
        """兼容旧接口：返回当前是否有可读状态（本实现恒为 True，状态由 action 驱动）。"""
        return True

    def has_cancelable_goal(self) -> bool:
        return self._task.is_active()

    def cancel(self) -> bool:
        return self._task.cancel()
