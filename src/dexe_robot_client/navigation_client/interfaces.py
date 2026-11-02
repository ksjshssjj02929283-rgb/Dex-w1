"""接口配置表：真实 / 仿真 / 迦智底盘配置。

sim 模式缺失的能力显式写 None——由 common 层（ActionTaskClient / call_service）
统一守卫，发送路径永不 AttributeError。

基线：dev_w1 分支（底盘精控走 action，路网导航/导航点查询是活跃接口，
由 W1Manager 直接调用，不可移除）。
"""

import os
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class InterfaceConfig:
    chassis_pose_topic: str
    battery_topic: str = None
    obstacle_topic: str = None
    nav2_action: str = None
    road_nav_action: str = None
    graph_points_nav_action: str = None
    nav_points_service: str = None
    precise_control_action: str = None
    auto_charging_action: str = None
    controller_param_node: str = None
    local_costmap_param_node: str = None
    global_costmap_param_node: str = None
    backend: str = "dexe"
    cloudia_host: str = None
    cloudia_task_port: int = 8002
    cloudia_token: str = None
    cloudia_poll_interval_s: float = 0.3
    cloudia_poll_fail_limit: int = 5
    cloudia_snap_xy_tol_m: float = 0.3
    cloudia_snap_yaw_tol_deg: float = 15.0
    cloudia_edges_bidirectional: bool = True
    cloudia_sched_ip: str = None
    cloudia_sched_port: int = 8858
    cloudia_sched_robot: int = 0
    cloudia_sched_map: str = None


REAL = InterfaceConfig(
    chassis_pose_topic="/feedback/chassis_pose",
    battery_topic="/diagnostics_agg",
    obstacle_topic="obstacle_detector/has_obstacle",
    nav2_action="navigate_to_pose",
    road_nav_action="/navigate_through_poses",
    graph_points_nav_action="/navigate_through_graph_points",
    nav_points_service="/map_asset_manager/get_all_nav_points",
    precise_control_action="control/precise_control",
    auto_charging_action="/auto_charging",
    controller_param_node="/controller_server",
    local_costmap_param_node="/local_costmap/local_costmap",
    global_costmap_param_node="/global_costmap/global_costmap",
)

# 迦智任务接口只替换客户端的任务/查询能力；实时速度、心跳、复位和清错
# 仍由底盘驱动侧负责。状态默认继续消费机器人服务端发布的 ROS 2 话题。
CLOUDIA = InterfaceConfig(
    chassis_pose_topic="/feedback/chassis_pose",
    battery_topic="/diagnostics_agg",
    obstacle_topic="obstacle_detector/has_obstacle",
    backend="cloudia",
    cloudia_host=os.environ.get("CLOUDIA_HOST"),
    cloudia_task_port=int(os.environ.get("CLOUDIA_TASK_PORT", "8002")),
    cloudia_token=os.environ.get("CLOUDIA_TOKEN"),
    cloudia_poll_interval_s=float(
        os.environ.get("CLOUDIA_POLL_INTERVAL_S", "0.3")),
    cloudia_poll_fail_limit=int(os.environ.get("CLOUDIA_POLL_FAIL_LIMIT", "5")),
    cloudia_snap_xy_tol_m=float(os.environ.get("CLOUDIA_SNAP_XY_TOL_M", "0.3")),
    cloudia_snap_yaw_tol_deg=float(
        os.environ.get("CLOUDIA_SNAP_YAW_TOL_DEG", "15.0")),
    cloudia_edges_bidirectional=os.environ.get("CLOUDIA_EDGES_BIDIRECTIONAL",
                                               "1").lower()
    not in {"0", "false", "no"},
)

# 仿真当前提供：底盘位姿反馈、精控 action，以及 controller_server 参数服务。
# 导航/路网/路径导航/回充/导航点在仿真下不可用，调用会安全返回 False/空结果，不崩溃。
SIM = InterfaceConfig(
    chassis_pose_topic="/feedback_sim/chassis_pose",
    precise_control_action="control/precise_control",
    controller_param_node="/controller_server",
)


def _with_sched(cfg: InterfaceConfig) -> InterfaceConfig:
    """迦智调度导航与底盘后端正交：配了 CLOUDIA_SCHED_IP 才启用。"""
    sched_ip = (os.environ.get("CLOUDIA_SCHED_IP") or "").strip()
    robot_raw = os.environ.get("CLOUDIA_SCHED_ROBOT", "").strip()
    if sched_ip and not robot_raw:
        raise ValueError(
            "已配置 CLOUDIA_SCHED_IP，必须同时指定 CLOUDIA_SCHED_ROBOT（调度车号）")
    try:
        robot = int(robot_raw) if robot_raw else 0
    except ValueError:
        raise ValueError(
            f"CLOUDIA_SCHED_ROBOT 应为正整数，收到：{robot_raw!r}") from None
    if sched_ip and robot <= 0:
        raise ValueError(f"CLOUDIA_SCHED_ROBOT 应为正整数，收到：{robot_raw!r}")
    return replace(
        cfg,
        cloudia_sched_ip=sched_ip or None,
        cloudia_sched_port=int(os.environ.get("CLOUDIA_SCHED_PORT", "8858")),
        cloudia_sched_robot=robot,
        cloudia_sched_map=os.environ.get("CLOUDIA_SCHED_MAP"),
    )


def get_interface_config(is_sim: bool = False) -> InterfaceConfig:
    """按配置选择后端，默认保持原有自研底盘行为。"""
    if is_sim:
        return SIM
    return _with_sched(_select_backend())


def _select_backend() -> InterfaceConfig:
    backend = os.environ.get("DEXE_CHASSIS_BACKEND", "dexe").lower()
    if backend == "cloudia":
        # 不依赖模块导入时的环境变量快照，允许测试或进程启动逻辑在构造
        # ChassisClient 前设置配置。
        return replace(
            CLOUDIA,
            cloudia_host=os.environ.get("CLOUDIA_HOST"),
            cloudia_task_port=int(os.environ.get("CLOUDIA_TASK_PORT", "8002")),
            cloudia_token=os.environ.get("CLOUDIA_TOKEN"),
            cloudia_poll_interval_s=float(
                os.environ.get("CLOUDIA_POLL_INTERVAL_S", "0.3")),
            cloudia_poll_fail_limit=int(
                os.environ.get("CLOUDIA_POLL_FAIL_LIMIT", "5")),
            cloudia_snap_xy_tol_m=float(
                os.environ.get("CLOUDIA_SNAP_XY_TOL_M", "0.3")),
            cloudia_snap_yaw_tol_deg=float(
                os.environ.get("CLOUDIA_SNAP_YAW_TOL_DEG", "15.0")),
            cloudia_edges_bidirectional=os.environ.get(
                "CLOUDIA_EDGES_BIDIRECTIONAL", "1").lower()
            not in {"0", "false", "no"},
        )
    return REAL
