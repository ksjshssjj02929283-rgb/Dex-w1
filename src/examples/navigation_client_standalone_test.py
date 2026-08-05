#!/usr/bin/env python3
"""navigation_client 独立验证驱动 —— 不依赖 W1Manager。

前置（终端 1）：python3 navigation_client_mock_server.py（同目录）
运行（终端 2）：
  source /opt/ros/humble/setup.bash && source ~/dexforce/8_interfaces/install/setup.bash
  export ROS_DOMAIN_ID=88   # ⚠️ 必须与 mock 同域且隔离——本机若有残留 nav2 进程
                            # （如 bt_navigator）会抢答 navigate_to_pose 导致假失败
  python3 navigation_client_standalone_test.py            # 全量（mock 场景）
  python3 navigation_client_standalone_test.py --only nav # 只跑某组：conn/nav/road/graph/points/ctrl/charge/param

对真实底盘验证：跳过 mock，直接在机器人网络里跑本脚本（导航/回充目标请先改成场地安全值）。

自管 rclpy 生命周期（等价于 W1Manager 的做法）：
  rclpy.init -> ChassisClient(is_sim=False) -> MultiThreadedExecutor 后台线程 spin
"""

import argparse
import os
import sys
import threading
import time

# 未 pip install 时从源码目录导入
_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

import rclpy
from rclpy.executors import MultiThreadedExecutor

from dexe_robot_client.navigation_client import ChassisClient

RESULTS = []


def check(name: str, ok: bool, detail: str = ""):
    RESULTS.append((name, ok, detail))
    print(f"  {'✅ PASS' if ok else '❌ FAIL'}  {name}" +
          (f"  ({detail})" if detail else ""))


def wait_until(pred, timeout: float, interval: float = 0.1) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(interval)
    return False


def test_connection(client):
    print("\n== 连接 / 状态读取 ==")
    check("is_connected", client.is_connected(timeout_sec=2.0))
    ok, pos = client.get_chassis_position()
    check("get_chassis_position", ok and pos is not None, f"pos={pos}")
    wait_until(lambda: client.get_battery_percentage()[0], 3.0)  # 电量 1Hz，等首帧
    ok, battery = client.get_battery_percentage()
    check("get_battery_percentage", ok and battery > 0, f"battery={battery}")
    check("get_front_obstacle 可读", client.get_front_obstacle() in (True, False))


def test_nav(client):
    print("\n== Nav2 导航 action ==")
    sent = client.set_chassis_nav2_position([1.0, 0.5, 90.0])
    check("导航目标发送", sent)
    if sent:
        check("状态进入 NAVIGATING",
              wait_until(lambda: client.get_nav_state() == "NAVIGATING", 2.0))
        check("状态到 SUCCEEDED（mock 3s 完成）",
              wait_until(lambda: client.get_nav_state() == "SUCCEEDED", 8.0),
              f"final={client.get_nav_state()}")

    sent = client.set_chassis_nav2_position([2.0, 0.5, 0.0])
    check("第二次导航目标发送", sent)
    if sent:
        time.sleep(0.5)  # 等 goal 被接受
        check("cancel_chassis_nav2 发出", client.cancel_chassis_nav2())
        check("状态到 CANCELLED",
              wait_until(lambda: client.get_nav_state() == "CANCELLED", 3.0),
              f"final={client.get_nav_state()}")


def test_road_navigation(client):
    print("\n== 路网 / 多路点导航 action ==")
    waypoints = [[1.0, 0.5, 0.0], [1.5, 0.5, 1.57]]  # yaw 单位：弧度
    sent = client.set_chassis_road_navigation(waypoints)
    check("路网导航目标发送", sent)
    if sent:
        check(
            "状态推进（ACCEPTED/EXECUTING）",
            wait_until(
                lambda: client.get_road_navigation_status() in
                ("ACCEPTED", "EXECUTING"), 3.0))
        check(
            "状态到 SUCCEEDED（mock 3s 完成）",
            wait_until(
                lambda: client.get_road_navigation_status() == "SUCCEEDED",
                8.0), f"final={client.get_road_navigation_status()}")
        ok, msg = client.get_road_navigation_result()
        check("路网导航结果读取", ok, f"msg={msg}")

    sent = client.set_chassis_road_navigation(waypoints)
    check("第二次路网导航目标发送", sent)
    if sent:
        time.sleep(0.5)
        check("cancel_road_navigation 发出", client.cancel_road_navigation())
        check(
            "状态到 CANCELED",
            wait_until(
                lambda: client.get_road_navigation_status() == "CANCELED", 3.0),
            f"final={client.get_road_navigation_status()}")
    check("空 waypoints 拒绝", client.set_chassis_road_navigation([]) is False)


def test_graph_points_navigation(client):
    print("\n== 路径导航 action（按点位名称）==")
    names = ["point_a", "point_b"]
    sent = client.set_chassis_graph_points_navigation(names)
    check("路径导航目标发送", sent)
    if sent:
        check(
            "状态推进（ACCEPTED/EXECUTING）",
            wait_until(
                lambda: client.get_graph_points_navigation_status() in
                ("ACCEPTED", "EXECUTING"), 3.0))
        check(
            "状态到 SUCCEEDED（mock 3s 完成）",
            wait_until(
                lambda: client.get_graph_points_navigation_status() ==
                "SUCCEEDED", 8.0),
            f"final={client.get_graph_points_navigation_status()}")
        ok, msg = client.get_graph_points_navigation_result()
        check("路径导航结果读取", ok, f"msg={msg}")

    sent = client.set_chassis_graph_points_navigation(names)
    check("第二次路径导航目标发送", sent)
    if sent:
        time.sleep(0.5)
        check("cancel_graph_points_navigation 发出",
              client.cancel_graph_points_navigation())
        check(
            "状态到 CANCELED",
            wait_until(
                lambda: client.get_graph_points_navigation_status() ==
                "CANCELED", 3.0),
            f"final={client.get_graph_points_navigation_status()}")


def test_nav_points(client):
    print("\n== 导航点查询 ==")
    points = client.get_all_nav_points()
    check("get_all_nav_points 非空", bool(points), f"points={points}")
    check("point_a 存在", "point_a" in points)


def test_ctrl(client):
    print("\n== 底盘精控 action（PreciseControl）==")
    sent = client.set_chassis_control_position([0.3, 0.0, 0.0])
    check("精控目标发送（x 平移）", sent)
    if sent:
        check("状态推进为 CONTROL（feedback 透传）",
              wait_until(lambda: client.get_control_status() == "CONTROL", 3.0),
              f"cur={client.get_control_status()}")
        check(
            "状态到 SUCCEEDED（mock 2s 完成，action result）",
            wait_until(lambda: client.get_control_status() == "SUCCEEDED", 6.0),
            f"final={client.get_control_status()}")

    sent = client.set_chassis_control_position([0.0, 0.0, 30.0])
    check("精控目标发送（yaw 旋转）", sent)
    if sent:
        wait_until(lambda: client.get_control_status() == "CONTROL", 3.0)
        check("cancel_chassis_control 发出", client.cancel_chassis_control())
        check(
            "状态到 CANCELLED",
            wait_until(lambda: client.get_control_status() == "CANCELLED", 3.0),
            f"final={client.get_control_status()}")
    check("非法输入拒绝（x 与 yaw 同时非零）",
          client.set_chassis_control_position([0.3, 0.0, 30.0]) is False)


def test_charge(client):
    print("\n== 自动回充 action ==")
    sent = client.set_chassis_auto_charging(force=True, timeout_sec=3.0)
    check("回充目标发送", sent)
    if sent:
        check(
            "状态推进（ACCEPTED/EXECUTING）",
            wait_until(
                lambda: client.get_auto_charging_status() in
                ("ACCEPTED", "EXECUTING"), 3.0))
        check(
            "状态到 SUCCEEDED（mock 3s 完成）",
            wait_until(lambda: client.get_auto_charging_status() == "SUCCEEDED",
                       8.0), f"final={client.get_auto_charging_status()}")
        ok, msg = client.get_auto_charging_result()
        check("回充结果读取", ok, f"msg={msg}")

    sent = client.set_chassis_auto_charging(force=True, timeout_sec=3.0)
    check("第二次回充目标发送", sent)
    if sent:
        time.sleep(0.5)
        check("cancel_auto_charging 发出", client.cancel_auto_charging())
        check(
            "状态到 CANCELED",
            wait_until(lambda: client.get_auto_charging_status() == "CANCELED",
                       3.0), f"final={client.get_auto_charging_status()}")


def test_param(client):
    print("\n== Nav2 参数服务（写后读校验）==")
    check("set_goal_tolerance", client.set_goal_tolerance(0.1, 0.15))
    check("set_inflation_radius", client.set_inflation_radius(0.45))
    check("set_follow_velocity_limits",
          client.set_follow_velocity_limits(0.5, 0.8))
    check(
        "set_footprint（4 顶点）",
        client.set_footprint([[0.3, 0.25], [0.3, -0.25], [-0.3, -0.25],
                              [-0.3, 0.25]]))


GROUPS = {
    "conn": test_connection,
    "nav": test_nav,
    "road": test_road_navigation,
    "graph": test_graph_points_navigation,
    "points": test_nav_points,
    "ctrl": test_ctrl,
    "charge": test_charge,
    "param": test_param,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--only",
                        choices=sorted(GROUPS),
                        default=None,
                        help="只跑一组测试")
    args = parser.parse_args()

    rclpy.init()
    client = ChassisClient(is_sim=False)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(client)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not wait_until(lambda: client.is_connected(timeout_sec=0.2), 5.0):
            print("❌ 5s 内未收到底盘位姿——请先启动 "
                  "navigation_client_mock_server.py（或确认真实底盘在线）")
            return 2
        groups = [args.only] if args.only else list(GROUPS)
        for name in groups:
            GROUPS[name](client)
    finally:
        print("\n================ 汇总 ================")
        failed = [r for r in RESULTS if not r[1]]
        for name, ok, detail in RESULTS:
            print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        print(f"共 {len(RESULTS)} 项：{len(RESULTS) - len(failed)} 过 / "
              f"{len(failed)} 失败")
        executor.shutdown()
        client.destroy_node()
        rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
