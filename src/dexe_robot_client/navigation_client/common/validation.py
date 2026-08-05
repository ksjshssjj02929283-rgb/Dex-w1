"""输入校验：所有目标位姿在进入 ROS 层之前统一检查。"""

import math


def parse_pose3(position):
    """校验 [x, y, yaw] 形输入。合法返回 [float, float, float]，否则返回 None。"""
    try:
        vals = [float(position[0]), float(position[1]), float(position[2])]
    except (TypeError, IndexError, ValueError, KeyError):
        return None
    if not all(math.isfinite(v) for v in vals):
        return None
    return vals
