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


def _integer(value, default=0):
    number = _finite(value)
    if number is None or not number.is_integer():
        return default
    return int(number)


def _parse_battery_state(raw, location):
    if not isinstance(raw, dict):
        return None

    voltage = _finite(raw.get("voltage"))
    temperature = _finite(raw.get("temperature"))
    current = _finite(raw.get("current"))
    charge = _finite(raw.get("charge"))
    capacity = _finite(raw.get("capacity"))
    design_capacity = _finite(raw.get("designCapacity"))
    percentage = _finite(raw.get("percentage"))
    if percentage is not None:
        if 0.0 <= percentage <= 1.0:
            pass
        elif 0.0 <= percentage <= 100.0:
            percentage /= 100.0
        else:
            percentage = None

    cell_voltages_raw = raw.get("cellVoltages")
    cell_voltages = []
    if isinstance(cell_voltages_raw, list):
        cell_voltages = [
            value for value in (_finite(item) for item in cell_voltages_raw)
            if value is not None
        ]

    known_values = (voltage, temperature, current, charge, capacity,
                    design_capacity, percentage)
    present_value = raw.get("present")
    has_known_value = any(value is not None for value in known_values)
    if not has_known_value and present_value is None and not cell_voltages:
        return None

    return {
        "voltage": voltage if voltage is not None else math.nan,
        "temperature": temperature if temperature is not None else math.nan,
        "current": current if current is not None else math.nan,
        "charge": charge if charge is not None else math.nan,
        "capacity": capacity if capacity is not None else math.nan,
        "design_capacity":
            (design_capacity if design_capacity is not None else math.nan),
        "percentage": percentage if percentage is not None else math.nan,
        "power_supply_status": _integer(raw.get("powerSupplyStatus")),
        "power_supply_health": _integer(raw.get("powerSupplyHealth")),
        "power_supply_technology": _integer(raw.get("powerSupplyTechnology")),
        "present": (bool(present_value)
                    if present_value is not None else has_known_value),
        "cell_voltage": cell_voltages,
        "location": location,
    }


def parse_upstream_state(payload):
    """解析一帧 upstream_state，字段不完整时返回 None 或缺失字段。

    返回值只包含已验证的标准化字段：pose、velocity、batteries、
    obstacle_stop。theta 使用弧度，BatteryState.percentage 使用 0-1。
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

    batteries = []
    for field, location in (("batteryState", "battery1"), ("battery2State",
                                                           "battery2")):
        battery = _parse_battery_state(state.get(field), location)
        if battery is not None:
            batteries.append(battery)

    information = payload.get("information") or {}
    return {
        "pose": pose,
        "velocity": velocity,
        "batteries": batteries,
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
    timestamp = datetime.now(
        timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
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
