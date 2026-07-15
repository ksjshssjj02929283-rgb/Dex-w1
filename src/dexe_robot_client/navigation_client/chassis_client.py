"""ChassisClient —— navigation_client 对外门面（重构 v2，基线 dev_w1）。

结构：interfaces.py（接口配置）+ common/（ActionTaskClient / 服务封装 / 运行时
/ 状态枚举，本模块内部通用机制）+ features/（Nav2 导航 / 路网导航 / 导航点查询 /
精控 / 回充 / 底盘状态 / 参数），本类只做组合与对外 API 兼容（方法签名与状态
字符串取值与 dev_w1 基线一致，W1Manager 零改动）。

运行模式：
- 默认（standalone=False）：与旧版一致，由外部 Executor（如 W1Manager）spin；
  独立使用时读接口内部的 spin_once 也能推进回调。
- standalone=True：自建后台执行器线程，用完调 close()。
"""

from rclpy.node import Node

from .common import runtime
from .features import (AutoChargingFeature, ChassisStateFeature,
                       Nav2NavigationFeature, Nav2ParamsFeature,
                       NavPointsFeature, PreciseControlFeature,
                       RoadNavigationFeature)
from .interfaces import REAL, SIM


class ChassisClient(Node):

    def __init__(self, is_sim: bool = False, timeout_sec: float = 1.0,
                 standalone: bool = False):
        """
        Args:
            is_sim: 仿真模式（当前仿真端仅提供位姿反馈与精控下发话题，
                导航/回充/参数接口调用会安全返回 False）。
            timeout_sec: 底盘位姿的连接超时阈值（秒）——超过该时长未收到
                位姿反馈即判定断连并清空位姿缓存。
            standalone: True 时自建后台执行器（独立使用免搭 executor 样板），
                需配对调用 close()。
        """
        self.node_name = "chassis_navigation" + ("_sim" if is_sim else "_real")
        super().__init__(self.node_name)
        cfg = SIM if is_sim else REAL
        self.get_logger().info(
            f"[底盘客户端创建] node={self.node_name}，"
            f"mode={'sim' if is_sim else 'real'}，standalone={standalone}")

        self._chassis_state = ChassisStateFeature(
            self, cfg, connection_timeout=timeout_sec)
        self._nav = Nav2NavigationFeature(self, cfg)
        self._road_nav = RoadNavigationFeature(self, cfg)
        self._nav_points = NavPointsFeature(self, cfg)
        self._control = PreciseControlFeature(self, cfg)
        self._charging = AutoChargingFeature(self, cfg)
        self._params = Nav2ParamsFeature(self, cfg)

        self._runner = runtime.StandaloneRunner(self) if standalone else None

    # ------------------------------------------------ 生命周期
    def close(self):
        """standalone 模式的显式释放；非 standalone 亦可调用（销毁节点）。"""
        if self._runner is not None:
            self._runner.close()
            self._runner = None
        self.destroy_node()

    def spin_once(self, timeout_sec: float = 0.1):
        runtime.spin_once(self, timeout_sec=timeout_sec)

    def spin(self):
        runtime.spin(self)

    # ------------------------------------------------ 连接 / 底盘状态
    def is_connected(self, timeout_sec: float = 0.1) -> bool:
        import time as _time
        deadline = _time.monotonic() + timeout_sec
        while True:
            self.spin_once(timeout_sec=0.02)
            if self._chassis_state.connected():
                return True
            if _time.monotonic() >= deadline:
                return False
            _time.sleep(0.01)

    def get_chassis_position(self):
        """返回 (success, np.ndarray([x, y, yaw_deg]))；断连后返回 (False, None)。"""
        self.spin_once()
        return self._chassis_state.get_position()

    def get_battery_percentage(self):
        """返回 (success, 0-100 电量)；未收到电量时 (False, -1.0)。"""
        self.spin_once()
        return self._chassis_state.battery()

    def get_front_obstacle(self) -> bool:
        self.spin_once()
        return self._chassis_state.obstacle()

    # ------------------------------------------------ Nav2 导航
    def set_chassis_nav2_position(self, position) -> bool:
        """发送导航目标（非阻塞）。position=[x, y, yaw]，yaw 单位：度，frame=map。"""
        return self._nav.send(position)

    def get_nav_state(self) -> str:
        """IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELLED（取值与旧版一致）。"""
        self.spin_once()
        return self._nav.state()

    def has_cancelable_nav_goal(self) -> bool:
        return self._nav.has_cancelable_goal()

    def cancel_chassis_nav2(self) -> bool:
        """取消导航（非阻塞）。goal 未被接受时登记挂起取消，接受后自动补发。"""
        return self._nav.cancel()

    # ------------------------------------------------ 路网 / 多路点导航
    def set_chassis_road_navigation(self, waypoints: list, frame_id: str = "map",
                                    timeout_sec: float = 10.0,
                                    behavior_tree: str = "") -> bool:
        """发送路网/多路点导航（非阻塞）。waypoints 为 [x, y, yaw_rad, ...] 列表，
        yaw 单位：弧度（与 Nav2 NavigateThroughPoses 原生接口一致）。"""
        return self._road_nav.send(waypoints, frame_id=frame_id,
                                   timeout_sec=timeout_sec,
                                   behavior_tree=behavior_tree)

    def get_road_navigation_status(self) -> str:
        self.spin_once()
        return self._road_nav.status()

    def get_road_navigation_result(self):
        return self._road_nav.result()

    def has_cancelable_road_navigation_goal(self) -> bool:
        return self._road_nav.has_cancelable_goal()

    def cancel_road_navigation(self) -> bool:
        return self._road_nav.cancel()

    # ------------------------------------------------ 导航点查询
    def get_all_nav_points(self) -> dict:
        """返回 dict[点位名称] -> (x, y, theta_rad)。"""
        return self._nav_points.get_all()

    # ------------------------------------------------ 底盘精控
    def set_chassis_control_position(self, position) -> bool:
        """发送精控目标（action）。position=[x, 0, yaw]，yaw 单位：度，x/yaw 互斥，frame=base_link。"""
        return self._control.send(position)

    def get_control_status(self, consume: bool = True) -> str:
        """IDLE / CONTROL / SUCCEEDED / CANCELLING / CANCELLED / FAILED（action 驱动）。"""
        self.spin_once()
        return self._control.status(consume=consume)

    def refresh_control_status(self) -> bool:
        return self._control.refresh()

    def has_cancelable_control_goal(self) -> bool:
        return self._control.has_cancelable_goal()

    def cancel_chassis_control(self) -> bool:
        return self._control.cancel()

    # ------------------------------------------------ 自动回充
    def set_chassis_auto_charging(self, force: bool = True,
                                  timeout_sec: float = 10.0) -> bool:
        return self._charging.start(force=force, timeout_sec=timeout_sec)

    def get_auto_charging_status(self) -> str:
        self.spin_once()
        return self._charging.status()

    def has_cancelable_auto_charging_goal(self) -> bool:
        return self._charging.has_cancelable_goal()

    def get_auto_charging_result(self):
        return self._charging.result()

    def cancel_auto_charging(self) -> bool:
        return self._charging.cancel()

    # ------------------------------------------------ Nav2 参数
    def set_goal_tolerance(self, xy_tolerance: float,
                           yaw_tolerance: float) -> bool:
        return self._params.set_goal_tolerance(xy_tolerance, yaw_tolerance)

    def set_footprint(self, vertices: list) -> bool:
        return self._params.set_footprint(vertices)

    def set_inflation_radius(self, radius: float) -> bool:
        return self._params.set_inflation_radius(radius)

    def set_follow_velocity_limits(self, vx_max: float, wz_max: float) -> bool:
        return self._params.set_follow_velocity_limits(vx_max, wz_max)
