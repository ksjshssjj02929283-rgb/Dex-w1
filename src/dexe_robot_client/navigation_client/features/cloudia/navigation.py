"""迦智单机导航与共享任务生命周期。"""

from __future__ import annotations

import math
import threading

from ...common import TaskState, parse_pose3
from .http import CloudiaApiError

_ROAD_LEGACY = {
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
    # 与原 RoadNavigationFeature 保持兼容：通信丢失对外返回 UNKNOWN。
    TaskState.LOST: "UNKNOWN",
}


class CloudiaTaskClient:
    """把 HTTP taskId + 轮询包装成现有 ActionTaskClient 的任务形状。"""

    _TERMINAL = frozenset({
        TaskState.SUCCEEDED,
        TaskState.CANCELED,
        TaskState.ABORTED,
        TaskState.REJECTED,
        TaskState.FAILED_TO_SEND,
        TaskState.LOST,
    })

    def __init__(self, node, api, cfg, *, label="迦智底盘任务"):
        self._node = node
        self._api = api
        self._label = label
        self._poll_interval = cfg.cloudia_poll_interval_s
        self._poll_fail_limit = cfg.cloudia_poll_fail_limit
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread = None
        self._generation = 0
        self._remote_task_id = None
        self._state = TaskState.IDLE
        self._result = (False, "")
        self._raw_task = None
        self._cancel_requested = False
        self._poll_failures = 0

    @property
    def state(self):
        with self._lock:
            return self._state

    def is_active(self):
        with self._lock:
            return self._state in {
                TaskState.SENDING,
                TaskState.ACCEPTED,
                TaskState.EXECUTING,
                TaskState.CANCELING,
            }

    def result(self):
        with self._lock:
            return self._result

    def start(self, task_type: str, *, trigger_ts=None, **fields) -> bool:
        with self._lock:
            if self.is_active():
                self._node.get_logger().warning(
                    f"[{self._label}] 已有活跃任务，拒绝并发下发。")
                return False
            self._generation += 1
            generation = self._generation
            self._state = TaskState.SENDING
            self._result = (False, "")
            self._raw_task = None
            self._cancel_requested = False
            self._poll_failures = 0

        try:
            payload = self._api.start_task(task_type, **fields)
            remote_task_id = payload.get("taskId")
            if remote_task_id is None:
                raise CloudiaApiError("任务受理响应缺少 taskId", payload=payload)
        except CloudiaApiError as exc:
            with self._lock:
                if generation == self._generation:
                    self._state = (TaskState.REJECTED if exc.code not in (None,
                                                                          1006)
                                   else TaskState.FAILED_TO_SEND)
                    self._result = (False, str(exc))
            self._node.get_logger().error(f"[{self._label}] 下发失败：{exc}")
            return False

        with self._lock:
            if generation != self._generation:
                return False
            self._remote_task_id = remote_task_id
            self._state = TaskState.ACCEPTED
            self._result = (False, "accepted")
            self._stop_event.clear()
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(
                    target=self._poll_loop,
                    name="cloudia-task-poll",
                    daemon=True,
                )
                self._thread.start()
        self._node.get_logger().info(
            f"[{self._label}] 任务已受理 taskId={remote_task_id}。")
        return True

    def _poll_loop(self):
        while not self._stop_event.is_set():
            self.refresh()
            self._stop_event.wait(self._poll_interval)

    def refresh(self):
        with self._lock:
            if self._remote_task_id is None or self._state in self._TERMINAL:
                return
            expected_task_id = self._remote_task_id
        try:
            payload = self._api.get_robot_status()
            status = payload.get("status", {})
            task = status.get("task")
            if task is None:
                task = payload.get("task")
            with self._lock:
                self._poll_failures = 0
                if not task or task.get("taskId") != expected_task_id:
                    self._node.get_logger().warning(
                        f"[{self._label}] 状态 taskId 不匹配，忽略本次响应："
                        f"期望={expected_task_id}，收到="
                        f"{None if task is None else task.get('taskId')}")
                    return
                self._apply_task_locked(task)
        except CloudiaApiError as exc:
            with self._lock:
                self._poll_failures += 1
                if self._poll_failures >= self._poll_fail_limit:
                    self._state = TaskState.LOST
                    self._result = (False, str(exc))
                    self._stop_event.set()
            self._node.get_logger().warning(f"[{self._label}] 状态查询失败：{exc}，"
                                            f"连续失败={self._poll_failures}")

    def _apply_task_locked(self, task):
        self._raw_task = task
        task_status = task.get("taskStatus")
        if task_status == "running":
            self._state = TaskState.EXECUTING
        elif task_status == "finished":
            self._state = TaskState.SUCCEEDED
            self._result = (True, "finished")
            self._stop_event.set()
        elif task_status == "failed":
            error = task.get("error") or {}
            message = error.get("errorMessage") or "迦智任务失败"
            if self._cancel_requested:
                self._state = TaskState.CANCELED
            else:
                self._state = TaskState.ABORTED
            self._result = (False, message)
            self._stop_event.set()

    def cancel(self) -> bool:
        with self._lock:
            if self._state not in {TaskState.ACCEPTED, TaskState.EXECUTING}:
                return False
            self._cancel_requested = True
            self._state = TaskState.CANCELING
        try:
            self._api.stop_task()
            return True
        except CloudiaApiError as exc:
            with self._lock:
                self._state = TaskState.ABORTED
                self._result = (False, str(exc))
            self._node.get_logger().error(f"[{self._label}] 取消失败：{exc}")
            return False

    def close(self):
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)


class CloudiaNavigationFeature:

    def __init__(self, node, task, points, state_reader, cfg):
        self._node = node
        self._task = task
        self._points = points
        self._state_reader = state_reader
        self._xy_tol = cfg.cloudia_snap_xy_tol_m
        self._yaw_tol = cfg.cloudia_snap_yaw_tol_deg
        self._edges_bidirectional = cfg.cloudia_edges_bidirectional

    @staticmethod
    def _angle_diff_deg(first, second):
        return abs((first - second + 180.0) % 360.0 - 180.0)

    def _nearest_xy(self, x, y):
        candidates = []
        for point in self._points.nodes():
            px, py, _ = point["pose"]
            candidates.append((math.hypot(x - px, y - py), point["id"]))
        if not candidates:
            raise ValueError("迦智地图没有可用点位")
        return min(candidates)

    def _snap_pose(self, position):
        values = parse_pose3(position)
        if values is None:
            raise ValueError("目标应为有限数值 [x, y, yaw_deg]")
        x, y, yaw_deg = values
        candidates = []
        for point in self._points.nodes():
            px, py, theta_rad = point["pose"]
            xy_error = math.hypot(x - px, y - py)
            yaw_error = self._angle_diff_deg(yaw_deg, math.degrees(theta_rad))
            candidates.append((xy_error, yaw_error, point["id"]))
        if not candidates:
            raise ValueError("迦智地图没有可用点位")
        xy_error, yaw_error, point_id = min(candidates)
        if xy_error > self._xy_tol or yaw_error > self._yaw_tol:
            raise ValueError(
                f"目标超出点位吸附容差：point={point_id}，"
                f"xy_error={xy_error:.3f}m，yaw_error={yaw_error:.1f}deg")
        return point_id

    def send(self, position, *, trigger_ts=None):
        try:
            point_id = self._snap_pose(position)
            return self._task.start("navigation",
                                    pointId=point_id,
                                    trigger_ts=trigger_ts)
        except (CloudiaApiError, TypeError, ValueError) as exc:
            self._node.get_logger().error(f"[迦智单点导航] {exc}")
            return False

    @staticmethod
    def _point_id(key):
        """点位键转迦智点位编号。

        接受整数、数字串、point_<编号>，以及调度路网的站点编号
        S<编号>（迦智路网同步到调度时按 S{点号:03d} 编号，如 S015）。
        其他类型的调度节点（L001、M001 等）不对应迦智点位，返回 None。
        """
        if isinstance(key, bool):
            return None
        if isinstance(key, int):
            return key
        text = str(key).strip()
        if text.startswith("point_"):
            text = text[len("point_"):]
        elif text[:1] == "S" and text[1:].isdigit():
            text = text[1:]
        return int(text) if text.isdigit() else None

    def _start_route(self, point_ids, *, trigger_ts=None):
        """坐标多点用：补齐首点、去重、校验连通后下发；首点必须是当前所在点位。"""
        ok, current_pose = self._state_reader()
        if not ok or current_pose is None:
            raise ValueError("无法获得当前底盘位姿，不能确定多点任务首点")
        _, current_id = self._nearest_xy(float(current_pose[0]),
                                         float(current_pose[1]))
        if current_id != point_ids[0]:
            point_ids = [current_id] + list(point_ids)
        compact = []
        for point_id in point_ids:
            if not compact or compact[-1] != point_id:
                compact.append(point_id)
        for source_id, dest_id in zip(compact, compact[1:]):
            if not self._points.is_connected(
                    source_id, dest_id,
                    bidirectional=self._edges_bidirectional):
                raise ValueError(f"点位不连通：{source_id} -> {dest_id}")
        if len(compact) == 1:
            return self._task.start("navigation",
                                    pointId=compact[0],
                                    trigger_ts=trigger_ts)
        return self._task.start("navigation",
                                pointIds=compact,
                                trigger_ts=trigger_ts)

    def send_road(self, waypoints, *, trigger_ts=None):
        if not waypoints:
            self._node.get_logger().error("[迦智多点导航] waypoints 为空。")
            return False
        try:
            # 既有路网接口 yaw 使用弧度；吸附比较统一使用度。
            point_ids = [
                self._snap_pose([x, y, math.degrees(yaw)])
                for x, y, yaw in waypoints
            ]
            return self._start_route(point_ids, trigger_ts=trigger_ts)
        except (CloudiaApiError, TypeError, ValueError) as exc:
            self._node.get_logger().error(f"[迦智多点导航] {exc}")
            return False

    def send_graph_points(self,
                          nav_point_names,
                          *,
                          nav_point_ids=None,
                          trigger_ts=None):
        """按点位名称 / 编号串接路径（交管下发的路径导航）。

        交管下发的编号是调度路网的站点编号（S015），名称是迦智点位名，
        点位没有名字时才是点号本身。同一位置优先用编号，解析不了再用名称。
        点列由交管规划，首点就是车当前所在点、相邻点连通，这里只校验点位
        存在，按点列原样下发，不补首点、不做连通校验。
        """
        names = list(nav_point_names or [])
        ids = list(nav_point_ids or [])
        keys = names if names else ids
        if not keys:
            self._node.get_logger().error("[迦智路径导航] 点位为空。")
            return False
        if names and ids and len(names) != len(ids):
            self._node.get_logger().error(
                "[迦智路径导航] nav_point_names 与 nav_point_ids 长度不一致。")
            return False
        try:
            known = {point["id"] for point in self._points.nodes()}
            point_ids = []
            for index, key in enumerate(keys):
                candidates = [key]
                if names and index < len(ids):
                    candidates.insert(0, ids[index])
                point_id = next((pid for pid in map(self._point_id, candidates)
                                 if pid is not None and pid in known), None)
                if point_id is None:
                    raise ValueError(f"迦智地图中找不到点位：{key}")
                point_ids.append(point_id)
        except (CloudiaApiError, TypeError, ValueError) as exc:
            self._node.get_logger().error(f"[迦智路径导航] {exc}")
            return False
        if len(point_ids) == 1:
            return self._task.start("navigation",
                                    pointId=point_ids[0],
                                    trigger_ts=trigger_ts)
        return self._task.start("navigation",
                                pointIds=point_ids,
                                trigger_ts=trigger_ts)

    def state(self):
        state = self._task.state
        if state in {
                TaskState.SENDING, TaskState.ACCEPTED, TaskState.EXECUTING,
                TaskState.CANCELING
        }:
            return "NAVIGATING"
        if state == TaskState.CANCELED:
            return "CANCELLED"
        if state in {
                TaskState.ABORTED, TaskState.REJECTED, TaskState.FAILED_TO_SEND,
                TaskState.LOST
        }:
            return "FAILED"
        return state.name

    def status(self):
        return _ROAD_LEGACY[self._task.state]

    def result(self):
        return self._task.result()

    def has_cancelable_goal(self):
        return self._task.is_active()

    def cancel(self):
        return self._task.cancel()
