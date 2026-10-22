"""迦智 WebSocket 消息编码与解析。

服务端只提取位姿和构造速度命令，其余状态随完整原始 JSON 转发。
"""

from __future__ import annotations

import math
import uuid
from datetime import datetime, timezone


def _finite(value, default=None):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return default
    return value if math.isfinite(value) else default


def parse_upstream_state(payload):
    """检查状态容器并提取位姿；不解析电池、障碍物或速度反馈。

    非 upstream_state、缺失/空/非法 state 返回 None，不刷新控制状态心跳。
    非空 state 对象即视为状态帧；位姿缺失或非法时返回 {"pose": None}。
    其它业务字段的有效性由原始 JSON 消费方判断。theta 保持弧度。
    """
    if not isinstance(payload, dict):
        return None
    if payload.get("communicationType") != "upstream_state":
        return None

    state = payload.get("state")
    if not isinstance(state, dict) or not state:
        return None
    position = state.get("agvPosition")
    if not isinstance(position, dict):
        return {"pose": None}
    pose = {
        "x": _finite(position.get("x")),
        "y": _finite(position.get("y")),
        "theta": _finite(position.get("theta")),
    }
    if any(value is None for value in pose.values()):
        pose = None
    return {"pose": pose}


def build_speed_command(vx,
                        vy,
                        omega,
                        *,
                        use_safe=False,
                        manufacturer="DexForce Cloudia Adapter",
                        serial_number="",
                        version="2.1.0"):
    """构造完整的 downstream_command/speedCommand 消息。"""
    timestamp = datetime.now(
        timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
    payload = {
        "communicationType": "downstream_command",
        "header": {
            "headerId": str(uuid.uuid4()),
            "timestamp": timestamp,
            "version": version,
            "manufacturer": manufacturer,
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
    # 协议允许省略头字段，但显式填写的字符串不能为空。
    if serial_number.strip():
        payload["header"]["serialNumber"] = serial_number
    return payload
