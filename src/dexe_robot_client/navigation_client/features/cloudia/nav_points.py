"""迦智地图点位与路网查询。"""

from __future__ import annotations

import math
import threading

from .http import CloudiaApiError


class CloudiaNavPointsFeature:

    def __init__(self, node, api):
        self._node = node
        self._api = api
        self._lock = threading.Lock()
        self._nodes = None
        self._points = None
        self._edges = set()

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        w = float(quaternion.get("w", 1.0))
        x = float(quaternion.get("x", 0.0))
        y = float(quaternion.get("y", 0.0))
        z = float(quaternion.get("z", 0.0))
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def refresh(self):
        payload = self._api.get_map_info()
        data = payload.get("data") or {}
        nodes = []
        points = {}
        for raw in data.get("nodes", []):
            try:
                point_id = int(raw["node_id"])
                pose = raw.get("pose") or {}
                position = pose.get("position") or {}
                orientation = pose.get("orientation") or {}
                value = (
                    float(position["x"]),
                    float(position["y"]),
                    self._yaw_from_quaternion(orientation),
                )
            except (KeyError, TypeError, ValueError) as exc:
                self._node.get_logger().warning(f"[迦智导航点跳过] 点位格式异常：{exc}")
                continue
            nodes.append({"id": point_id, "pose": value})
            # 当前文档只给出 node_id，业务名称映射待后续配置注入。
            points[f"point_{point_id}"] = value
        edges = set()
        for raw in data.get("edges", []):
            try:
                edges.add((int(raw["source_node"]), int(raw["dest_node"])))
            except (KeyError, TypeError, ValueError):
                continue
        with self._lock:
            self._nodes = nodes
            self._points = points
            self._edges = edges
        return dict(points)

    def get_all(self):
        with self._lock:
            if self._points is not None:
                return dict(self._points)
        try:
            return self.refresh()
        except CloudiaApiError as exc:
            self._node.get_logger().error(f"[迦智导航点查询失败] {exc}")
            return {}

    def nodes(self):
        with self._lock:
            cached = self._nodes
        if cached is None:
            self.refresh()
            with self._lock:
                cached = self._nodes
        return list(cached or [])

    def is_connected(self,
                     source_id: int,
                     dest_id: int,
                     *,
                     bidirectional: bool = True):
        with self._lock:
            if self._nodes is None:
                cached = None
            else:
                cached = self._edges
        if cached is None:
            self.refresh()
            with self._lock:
                cached = self._edges
        return ((source_id, dest_id) in cached or
                (bidirectional and (dest_id, source_id) in cached))
