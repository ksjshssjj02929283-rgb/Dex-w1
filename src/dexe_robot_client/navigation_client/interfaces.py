"""接口配置表：真实 / 仿真两份声明式配置。

sim 模式缺失的能力显式写 None——由 common 层（ActionTaskClient / call_service）
统一守卫，发送路径永不 AttributeError。

基线：dev_w1 分支（底盘精控走 action，路网导航/导航点查询是活跃接口，
由 W1Manager 直接调用，不可移除）。
"""

from dataclasses import dataclass


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

# 仿真当前提供：底盘位姿反馈、精控 action，以及 controller_server 参数服务。
# 导航/路网/路径导航/回充/导航点在仿真下不可用，调用会安全返回 False/空结果，不崩溃。
SIM = InterfaceConfig(
    chassis_pose_topic="/feedback_sim/chassis_pose",
    precise_control_action="control/precise_control",
    controller_param_node="/controller_server",
)
