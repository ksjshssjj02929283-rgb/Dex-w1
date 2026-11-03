"""ChassisClient —— navigation_client 对外门面（重构 v2，基线 dev_w1）。

结构：interfaces.py（接口配置）+ common/（ActionTaskClient / 服务封装 / 运行时
/ 状态枚举，本模块内部通用机制）+ features/（Nav2 导航 / 路网导航 / 路径导航 /
导航点查询 / 精控 / 回充 / 底盘状态 / 参数），本类只做组合与对外 API 门面。既有
方法签名和状态字符串保持兼容；迦智后端按 DEXE_CHASSIS_BACKEND 选装，自研路径不变。

运行模式：
- 默认（standalone=False）：与旧版一致，由外部 Executor（如 W1Manager）spin；
  独立使用时读接口内部的 spin_once 也能推进回调。
- standalone=True：自建后台执行器线程，用完调 close()。

延迟诊断：五个"触发类"公开方法（set_chassis_nav2_position / set_chassis_
road_navigation / set_chassis_graph_points_navigation /
set_chassis_control_position / set_chassis_auto_charging）
入口处打印链路起点"触发"检查点（挂钟时间），这是"上层 application 调用
本客户端"与"客户端实际收到调用"之间可比较的最早时间点——与上层自己记录
的调用时刻相减，即知道 app→client 这一段延迟。触发时刻会传给对应
feature 的 send()/start()，串成同一个 task_id 上的检查点链：
**发送链路**（触发→server就绪→已发出→accepted，见 chassis_client 本身
与 common/action_task_client.py）+ **反馈链路**（进度→result，同样在
action_task_client.py）。全链路统一用 `common/timing.format_checkpoint()`
生成日志行，格式完全一致（阶段名 + 挂钟时间 + 相对上一检查点增量 +
相对触发的累计耗时）。
"""

from rclpy.node import Node

from .common import runtime, timing
from .features import (AutoChargingFeature, ChassisStateFeature,
                       CloudiaHttpClient, CloudiaNavPointsFeature,
                       CloudiaNavigationFeature, CloudiaPreciseControlFeature,
                       CloudiaSchedHttpClient, CloudiaSchedNavigationFeature,
                       CloudiaTaskClient, GraphPointsNavigationFeature,
                       Nav2NavigationFeature, Nav2ParamsFeature,
                       NavPointsFeature, PreciseControlFeature,
                       RoadNavigationFeature)
from .features.cloudia.scheduler import STOP_CHARGE_ACTION
from .interfaces import get_interface_config


class ChassisClient(Node):

    def __init__(self,
                 is_sim: bool = False,
                 timeout_sec: float = 1.0,
                 standalone: bool = False):
        """
        Args:
            is_sim: 仿真模式（当前提供位姿反馈、精控 action 和 controller
                参数；导航/路网/路径导航/回充调用会安全返回 False）。
            timeout_sec: 底盘位姿的连接超时阈值（秒）——超过该时长未收到
                位姿反馈即判定断连并清空位姿缓存。
            standalone: True 时自建后台执行器（独立使用免搭 executor 样板），
                需配对调用 close()。
        """
        self.node_name = "chassis_navigation" + ("_sim" if is_sim else "_real")
        super().__init__(self.node_name)
        cfg = get_interface_config(is_sim)
        self._backend = cfg.backend
        self.get_logger().info(
            f"[底盘客户端创建] node={self.node_name}，"
            f"mode={'sim' if is_sim else 'real'}，standalone={standalone}")

        self._cloudia_api = None
        self._cloudia_task = None
        self._cloudia_nav = None
        self._cloudia_control = None
        self._cloudia_points = None

        # 迦智后端的位姿、电量同样来自 ROS 2 话题（由 robot_server 适配器发布）。
        self._chassis_state = ChassisStateFeature(
            self, cfg, connection_timeout=timeout_sec)
        if cfg.backend == "cloudia":
            self._cloudia_api = CloudiaHttpClient(self, cfg)
            self._cloudia_task = CloudiaTaskClient(self, self._cloudia_api, cfg)
            self._cloudia_points = CloudiaNavPointsFeature(
                self, self._cloudia_api)
            self._cloudia_nav = CloudiaNavigationFeature(
                self, self._cloudia_task, self._cloudia_points,
                self._chassis_state.get_position, cfg)
            self._cloudia_control = CloudiaPreciseControlFeature(
                self, self._cloudia_task)
        self._sched_nav = (CloudiaSchedNavigationFeature(
            self,
            CloudiaSchedHttpClient(
                f"http://{cfg.cloudia_sched_ip}:{cfg.cloudia_sched_port}"), cfg)
                           if cfg.cloudia_sched_ip else None)
        self._sched_send_error = ""
        self._nav = Nav2NavigationFeature(self, cfg)
        self._road_nav = RoadNavigationFeature(self, cfg)
        self._graph_points_nav = GraphPointsNavigationFeature(self, cfg)
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
        if self._cloudia_task is not None:
            self._cloudia_task.close()
        if self._sched_nav is not None:
            self._sched_nav.close()
        self.destroy_node()

    def spin_once(self, timeout_sec: float = 0.1):
        runtime.spin_once(self, timeout_sec=timeout_sec)

    def spin(self):
        runtime.spin(self)

    def _log_trigger(self, label: str) -> float:
        """延迟诊断【链路起点】：打印客户端方法被调用的挂钟时刻（阶段=触发）。
        返回值需传给对应 feature 的 send()/start()（trigger_ts 参数），作为
        该任务后续所有检查点的"累计"计算基准。

        本行始终打印（不受诊断开关影响），格式随 timing.diagnostics_enabled()
        切换：打开=完整检查点格式（[延迟诊断] 标签，与下游检查点一致）；
        关闭（默认）=普通日志，只给挂钟时间，配合 result 的"返回"行看总耗时。
        """
        ts = timing.now()
        if timing.diagnostics_enabled():
            self.get_logger().info(timing.format_checkpoint(label, "触发", ts))
        else:
            self.get_logger().info(timing.format_plain_trigger(label, ts))
        return ts

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
        trigger_ts = self._log_trigger("Nav2 导航")
        if self._cloudia_nav is not None:
            return self._cloudia_nav.send(position, trigger_ts=trigger_ts)
        return self._nav.send(position, trigger_ts=trigger_ts)

    def get_nav_state(self) -> str:
        """IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELLED（取值与旧版一致）。"""
        self.spin_once()
        if self._cloudia_nav is not None:
            return self._cloudia_nav.state()
        return self._nav.state()

    def has_cancelable_nav_goal(self) -> bool:
        if self._cloudia_nav is not None:
            return self._cloudia_nav.has_cancelable_goal()
        return self._nav.has_cancelable_goal()

    def cancel_chassis_nav2(self) -> bool:
        """取消导航（非阻塞）。goal 未被接受时登记挂起取消，接受后自动补发。"""
        if self._cloudia_nav is not None:
            return self._cloudia_nav.cancel()
        return self._nav.cancel()

    # ------------------------------------------------ 迦智调度导航
    def set_chassis_sched_target(self,
                                 id=None,
                                 name=None,
                                 position=None,
                                 action="") -> bool:
        """经迦智调度导航到目标点位。

        返回 False 表示没下发成功，原因用 get_sched_send_error() 取；
        下发成功后的执行结果用 get_sched_nav_state() / get_sched_nav_result() 查。

        Args:
            id: 点位 id，仅在 name 为 None 时使用，原样下发。
            name: 点位名称(str)，优先于 id。迦智底盘的名称就是点位 id 的
                字符串形式，这里转成 int 下发。
            position: 目标坐标，迦智调度按点位导航，暂不使用。
            action: 到点后执行的车端动作，空表示无动作。充电：充电点 id +
                action=CHARGE_ACTION。
        id 和 name 不能同时为 None。
        """
        del position
        if self._sched_nav is None:
            return self._sched_send_fail("未配置 CLOUDIA_SCHED_IP")
        if id is None and name is None:
            return self._sched_send_fail("id 和 name 不能同时为 None")
        if name is not None:
            try:
                id = int(str(name).strip())
            except ValueError:
                return self._sched_send_fail(f"name 应为整数点位 id，收到：{name!r}")
        trigger_ts = self._log_trigger("迦智调度导航")
        return self._sched_send(id, action=action, trigger_ts=trigger_ts)

    def set_chassis_sched_stop_charge(self) -> bool:
        """经迦智调度原地执行退桩动作；状态同样用 get_sched_nav_state 查。

        返回 False 时原因用 get_sched_send_error() 取。
        """
        if self._sched_nav is None:
            return self._sched_send_fail("未配置 CLOUDIA_SCHED_IP")
        trigger_ts = self._log_trigger("迦智调度退桩")
        return self._sched_send(0,
                                action=STOP_CHARGE_ACTION,
                                trigger_ts=trigger_ts)

    def _sched_send(self, target, **kwargs) -> bool:
        ok = self._sched_nav.send(target, **kwargs)
        self._sched_send_error = "" if ok else (
            self._sched_nav.last_send_error() or "迦智调度下发失败（原因未知）")
        return ok

    def _sched_send_fail(self, reason: str) -> bool:
        self._sched_send_error = reason
        self.get_logger().error(f"[迦智调度导航] {reason}。")
        return False

    def get_sched_send_error(self) -> str:
        """最近一次迦智调度下发（set_chassis_sched_target / set_chassis_sched_stop_charge）
        返回 False 的原因，例如 CLOUDIA 的 400 响应原文、并发拒绝、连不上调度；
        最近一次下发成功则为空串。"""
        return self._sched_send_error

    def get_sched_nav_state(self) -> str:
        if self._sched_nav is None:
            return "IDLE"
        return self._sched_nav.state()

    def get_sched_nav_result(self):
        if self._sched_nav is None:
            return False, ""
        return self._sched_nav.result()

    def has_cancelable_sched_goal(self) -> bool:
        return self._sched_nav is not None and self._sched_nav.has_cancelable_goal(
        )

    def cancel_sched_nav(self) -> bool:
        return self._sched_nav is not None and self._sched_nav.cancel()

    # ------------------------------------------------ 路网 / 多路点导航
    def set_chassis_road_navigation(self,
                                    waypoints: list,
                                    frame_id: str = "map",
                                    timeout_sec: float = 10.0,
                                    behavior_tree: str = "") -> bool:
        """发送路网/多路点导航（非阻塞）。waypoints 为 [x, y, yaw_rad, ...] 列表，
        yaw 单位：弧度（与 Nav2 NavigateThroughPoses 原生接口一致）。"""
        trigger_ts = self._log_trigger("路网导航")
        if self._cloudia_nav is not None:
            return self._cloudia_nav.send_road(waypoints, trigger_ts=trigger_ts)
        return self._road_nav.send(waypoints,
                                   frame_id=frame_id,
                                   timeout_sec=timeout_sec,
                                   behavior_tree=behavior_tree,
                                   trigger_ts=trigger_ts)

    def get_road_navigation_status(self) -> str:
        self.spin_once()
        if self._cloudia_nav is not None:
            return self._cloudia_nav.status()
        return self._road_nav.status()

    def get_road_navigation_result(self):
        if self._cloudia_nav is not None:
            return self._cloudia_nav.result()
        return self._road_nav.result()

    def has_cancelable_road_navigation_goal(self) -> bool:
        if self._cloudia_nav is not None:
            return self._cloudia_nav.has_cancelable_goal()
        return self._road_nav.has_cancelable_goal()

    def cancel_road_navigation(self) -> bool:
        if self._cloudia_nav is not None:
            return self._cloudia_nav.cancel()
        return self._road_nav.cancel()

    # ------------------------------------------------ 路径导航（按点位名称）
    def set_chassis_graph_points_navigation(self,
                                            nav_point_names: list,
                                            nav_point_ids: list = None,
                                            pose=None,
                                            target_yaw: float = None,
                                            timeout_sec: float = 10.0) -> bool:
        """发送路径导航目标（非阻塞）。按 `nav_point_names` 串接路径，由服务端
        解析路点并逐段执行；不校验点位数量、不拦截并发。
        迦智后端按点位编号下发多点任务，pose / target_yaw 不参与。"""
        trigger_ts = self._log_trigger("路径导航")
        if self._cloudia_nav is not None:
            if pose is not None or target_yaw is not None:
                self.get_logger().warning("[路径导航] 迦智后端不使用 pose / target_yaw。")
            return self._cloudia_nav.send_graph_points(
                nav_point_names,
                nav_point_ids=nav_point_ids,
                trigger_ts=trigger_ts)
        return self._graph_points_nav.send(nav_point_names,
                                           nav_point_ids=nav_point_ids,
                                           pose=pose,
                                           target_yaw=target_yaw,
                                           timeout_sec=timeout_sec,
                                           trigger_ts=trigger_ts)

    def get_graph_points_navigation_status(self) -> str:
        self.spin_once()
        if self._cloudia_nav is not None:
            return self._cloudia_nav.status()
        return self._graph_points_nav.status()

    def get_graph_points_navigation_result(self):
        if self._cloudia_nav is not None:
            return self._cloudia_nav.result()
        return self._graph_points_nav.result()

    def has_cancelable_graph_points_navigation_goal(self) -> bool:
        if self._cloudia_nav is not None:
            return self._cloudia_nav.has_cancelable_goal()
        return self._graph_points_nav.has_cancelable_goal()

    def cancel_graph_points_navigation(self) -> bool:
        if self._cloudia_nav is not None:
            return self._cloudia_nav.cancel()
        return self._graph_points_nav.cancel()

    # ------------------------------------------------ 导航点查询
    def get_all_nav_points(self) -> dict:
        """返回 dict[点位名称] -> (x, y, theta_rad)。"""
        if self._cloudia_points is not None:
            return self._cloudia_points.get_all()
        return self._nav_points.get_all()

    # ------------------------------------------------ 底盘精控
    def set_chassis_control_position(self, position) -> bool:
        """发送精控目标（action）。position=[x, 0, yaw]，yaw 单位：度，x/yaw 互斥，frame=base_link。"""
        trigger_ts = self._log_trigger("底盘精控")
        if self._cloudia_control is not None:
            return self._cloudia_control.send(position, trigger_ts=trigger_ts)
        return self._control.send(position, trigger_ts=trigger_ts)

    def get_control_status(self, consume: bool = True) -> str:
        """IDLE / CONTROL / SUCCEEDED / CANCELLING / CANCELLED / FAILED（action 驱动）。"""
        self.spin_once()
        if self._cloudia_control is not None:
            return self._cloudia_control.status(consume=consume)
        return self._control.status(consume=consume)

    def refresh_control_status(self) -> bool:
        if self._cloudia_control is not None:
            return self._cloudia_control.refresh()
        return self._control.refresh()

    def has_cancelable_control_goal(self) -> bool:
        if self._cloudia_control is not None:
            return self._cloudia_control.has_cancelable_goal()
        return self._control.has_cancelable_goal()

    def cancel_chassis_control(self) -> bool:
        if self._cloudia_control is not None:
            return self._cloudia_control.cancel()
        return self._control.cancel()

    # ------------------------------------------------ 自动回充
    def set_chassis_auto_charging(self,
                                  force: bool = True,
                                  timeout_sec: float = 10.0) -> bool:
        trigger_ts = self._log_trigger("自动回充")
        return self._charging.start(force=force,
                                    timeout_sec=timeout_sec,
                                    trigger_ts=trigger_ts)

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
