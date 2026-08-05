"""Nav2 参数读写：controller / local_costmap / global_costmap 的 set + 写后读校验。"""

import math
import time

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter

from ..common import call_service


def _param_to_python(value):
    t = value.type
    if t == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if t == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if t == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if t == ParameterType.PARAMETER_STRING:
        return value.string_value
    return None


def _values_match(expected, retrieved) -> bool:
    if isinstance(retrieved, float):
        return math.isclose(retrieved, float(expected), rel_tol=1e-6)
    if isinstance(retrieved, bool):
        return retrieved == bool(expected)
    if isinstance(retrieved, int):
        return retrieved == int(expected)
    if isinstance(retrieved, str):
        return retrieved == str(expected)
    return False


class Nav2ParamsFeature:

    def __init__(self, node, cfg):
        self._node = node

        def clients(param_node):
            if not param_node:
                return None, None
            return (node.create_client(SetParameters,
                                       f"{param_node}/set_parameters"),
                    node.create_client(GetParameters,
                                       f"{param_node}/get_parameters"))

        self._controller_set, self._controller_get = clients(
            cfg.controller_param_node)
        self._local_set, self._local_get = clients(cfg.local_costmap_param_node)
        self._global_set, self._global_get = clients(
            cfg.global_costmap_param_node)

    # ------------------------------------------------ 公共接口
    def set_goal_tolerance(self, xy_tolerance: float,
                           yaw_tolerance: float) -> bool:
        """更新当前 GoalChecker 支持的容差；yaw_tolerance 单位为弧度。"""
        if not self._positive_finite(xy_tolerance) or not self._positive_finite(
                yaw_tolerance):
            self._node.get_logger().error("[目标容差非法] xy/yaw 必须为大于 0 的有限数值。")
            return False

        active_checker = self._get_param(self._controller_get,
                                         "current_goal_checker",
                                         quiet=True)
        if not isinstance(active_checker, str) or not active_checker:
            self._node.get_logger().error(
                "[目标容差写入失败] 无法确定 current_goal_checker。")
            return False

        candidate_values = {
            f"{active_checker}.xy_goal_tolerance": xy_tolerance,
            f"{active_checker}.lon_goal_tolerance": xy_tolerance,
            f"{active_checker}.lat_goal_tolerance": xy_tolerance,
            f"{active_checker}.yaw_goal_tolerance": yaw_tolerance,
        }
        available = {
            name: value for name, value in candidate_values.items() if
            self._get_param(self._controller_get, name, quiet=True) is not None
        }

        xy_name = f"{active_checker}.xy_goal_tolerance"
        lon_name = f"{active_checker}.lon_goal_tolerance"
        lat_name = f"{active_checker}.lat_goal_tolerance"
        yaw_name = f"{active_checker}.yaw_goal_tolerance"
        if (lon_name in available) != (lat_name in available):
            self._node.get_logger().error(
                f"[目标容差写入失败] {active_checker} 的 lon/lat 参数不完整。")
            return False
        if not any(name in available for name in (xy_name, lon_name, yaw_name)):
            self._node.get_logger().error(
                f"[目标容差写入失败] {active_checker} 未提供受支持的容差参数。")
            return False

        ordered_names = (xy_name, lon_name, lat_name, yaw_name)
        results = [
            self._set_param(self._controller_set, self._controller_get, name,
                            available[name])
            for name in ordered_names
            if name in available
        ]
        return bool(results) and all(results)

    def set_footprint(self, vertices: list) -> bool:
        if not isinstance(vertices, (list, tuple)) or len(vertices) < 4:
            self._node.get_logger().error("[footprint 非法] 至少需要 4 个 [x, y] 顶点。")
            return False
        if any(not isinstance(vertex, (list, tuple)) or len(vertex) != 2 or any(
                not self._finite_number(value)
                for value in vertex)
               for vertex in vertices):
            self._node.get_logger().error(
                "[footprint 非法] 每个顶点必须是两个有限数值组成的 [x, y]。")
            return False
        footprint = "[" + ",".join(f"[{v[0]},{v[1]}]" for v in vertices) + "]"
        return (self._set_param(self._local_set, self._local_get, 'footprint',
                                footprint) and
                self._set_param(self._global_set, self._global_get, 'footprint',
                                footprint))

    def set_inflation_radius(self, radius: float) -> bool:
        return (self._set_param(self._local_set, self._local_get,
                                'inflation_layer.inflation_radius', radius) and
                self._set_param(self._global_set, self._global_get,
                                'inflation_layer.inflation_radius', radius))

    def set_follow_velocity_limits(self, vx_max: float, wz_max: float) -> bool:
        return (self._set_param(self._controller_set, self._controller_get,
                                'FollowPath.vx_max', vx_max) and
                self._set_param(self._controller_set, self._controller_get,
                                'FollowPath.wz_max', wz_max))

    # ------------------------------------------------ 内部
    @staticmethod
    def _finite_number(value) -> bool:
        return (isinstance(value, (int, float)) and
                not isinstance(value, bool) and math.isfinite(float(value)))

    @classmethod
    def _positive_finite(cls, value) -> bool:
        return cls._finite_number(value) and float(value) > 0.0

    def _get_param(self, get_client, name: str, quiet: bool = False):
        request = GetParameters.Request()
        request.names = [name]
        response = call_service(self._node,
                                get_client,
                                request,
                                timeout_sec=1.0,
                                label=f"参数读取 {name}")
        if response is None or not response.values:
            if not quiet:
                self._node.get_logger().error(f"[参数读取失败] {name}：读取失败。")
            return None
        return _param_to_python(response.values[0])

    def _set_param(self, set_client, get_client, name: str, value) -> bool:
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
        ]
        response = call_service(self._node,
                                set_client,
                                request,
                                timeout_sec=1.0,
                                label=f"参数写入 {name}")
        if response is None or not response.results[0].successful:
            reason = (response.results[0].reason if response else "调用失败")
            self._node.get_logger().error(f"[参数写入失败] {name}：{reason}")
            return False

        time.sleep(0.1)  # 等服务端应用后再读校验
        retrieved = self._get_param(get_client, name)
        if retrieved is None or not _values_match(value, retrieved):
            self._node.get_logger().error(
                f"[参数校验失败] {name}：期望 {value}，实际 {retrieved}。")
            return False
        self._node.get_logger().info(f"[参数已写入并校验] {name} = {value}")
        return True
