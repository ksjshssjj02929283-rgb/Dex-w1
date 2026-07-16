"""自动回充功能：ActionTaskClient(AutoCharging)。状态取值与旧版一致。"""

from chassis_interfaces.action import AutoCharging

from ..common import ActionTaskClient, TaskState


def _feedback_summary(feedback) -> str:
    return f"状态={feedback.current_state}，重试次数={feedback.retry_count}"


class AutoChargingFeature:

    def __init__(self, node, cfg):
        self._task = ActionTaskClient(
            node, AutoCharging, cfg.auto_charging_action,
            label="自动回充", wait_server_timeout=10.0,
            feedback_formatter=_feedback_summary)

    def start(self, force: bool = True, timeout_sec: float = 10.0, *,
             trigger_ts: float = None) -> bool:
        goal = AutoCharging.Goal()
        goal.force = force
        return self._task.send(goal, wait_server_timeout=timeout_sec,
                               trigger_ts=trigger_ts)

    def status(self) -> str:
        state = self._task.state
        if state == TaskState.LOST:
            return "UNKNOWN"
        return state.name  # 与旧版取值一致（含 IDLE/…/FAILED_TO_SEND/UNKNOWN）

    def has_cancelable_goal(self) -> bool:
        return self._task.is_active()

    def result(self):
        return self._task.result()

    def cancel(self) -> bool:
        return self._task.cancel()
