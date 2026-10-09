"""迦智 WebSocket 消息编码与解析。

协议字段只保留在服务端厂商适配包内；ROS 节点不向上层暴露原始 JSON。
"""

from __future__ import annotations

import math
import uuid
from datetime import datetime, timezone


def _finite(value, default=None):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def parse_upstream_state(payload):
    """解析一帧 upstream_state，字段不完整时返回 None 或缺失字段。

    返回值只包含已验证的标准化字段：pose、velocity、battery_percentage、
    obstacle_stop。theta 使用弧度，battery_percentage 使用 0-100。
    """
    if not isinstance(payload, dict):
        return None
    if payload.get("communicationType") != "upstream_state":
        return None

    state = payload.get("state") or {}
    position = state.get("agvPosition") or {}
    pose = {
        "x": _finite(position.get("x")),
        "y": _finite(position.get("y")),
        "theta": _finite(position.get("theta")),
    }
    if any(value is None for value in pose.values()):
        pose = None

    velocity_raw = state.get("velocity") or {}
    velocity = {
        "vx": _finite(velocity_raw.get("vx"), 0.0),
        "vy": _finite(velocity_raw.get("vy"), 0.0),
        "omega": _finite(velocity_raw.get("omega"), 0.0),
    }

    battery_raw = state.get("batteryState") or {}
    battery_fraction = _finite(battery_raw.get("percentage"))
    battery_percentage = None
    if battery_fraction is not None:
        battery_percentage = (battery_fraction * 100.0
                              if 0.0 <= battery_fraction <= 1.0 else
                              battery_fraction)

    information = payload.get("information") or {}
    return {
        "pose": pose,
        "velocity": velocity,
        "battery_percentage": battery_percentage,
        "obstacle_stop": bool(information.get("obstacleStop", False)),
        "emergency": bool(information.get("emergency", False)),
        "raw_header": payload.get("header") or {},
    }


def build_speed_command(vx,
                        vy,
                        omega,
                        *,
                        use_safe=False,
                        manufacturer="DexForce Cloudia Adapter",
                        serial_number="",
                        version="2.1.0"):
    """构造完整的 downstream_command/speedCommand 消息。"""
    timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z")
    return {
        "communicationType": "downstream_command",
        "header": {
            "headerId": str(uuid.uuid4()),
            "timestamp": timestamp,
            "version": version,
            "manufacturer": manufacturer,
            "serialNumber": serial_number,
        },
        "speedCommand": {
            "targetVelocity": {
                "vx": float(vx),
                "vy": float(vy),
                "omega": float(omega),
            },
            "useSafe": bool(use_safe),
        },
    }
