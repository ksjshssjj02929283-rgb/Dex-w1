#!/usr/bin/env python3
"""Automatic interface self-test shipped inside ``navigation_client``.

The runner intentionally contains no physical-robot motion or charging cases.
Those cases require an approved manual plan and are documented separately.

The canonical PC2 invocation uses ``python3 -m`` because the current PC2
deployment copies this package directory directly into ``site-packages``.
The shorter ``dexe-navigation-client-test`` alias exists only after a package
installation step has generated the console script.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
import unicodedata
from dataclasses import asdict, dataclass, field
from datetime import datetime
from importlib import metadata
from pathlib import Path
from typing import Callable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from .chassis_client import ChassisClient
from .test_support.mock_chassis import MockChassisServer, make_parameter_nodes


@dataclass(frozen=True)
class CaseDefinition:
    case_id: str
    name: str
    purpose: str


CASES = (
    CaseDefinition("RC-A-01", "底盘状态读取", "验证连接、位姿、电量和障碍物缓存"),
    CaseDefinition("RC-A-02", "位姿断连与恢复", "验证超时清缓存及重新连接"),
    CaseDefinition("RC-A-03", "Nav2 导航成功", "验证合法目标发送、状态和终态"),
    CaseDefinition("RC-A-04", "Nav2 导航取消", "验证可取消状态和取消闭环"),
    CaseDefinition("RC-A-05", "导航点查询", "验证服务响应到公开字典的转换"),
    CaseDefinition("RC-A-06", "底盘精控成功", "验证平移目标、反馈状态和终态"),
    CaseDefinition("RC-A-07", "底盘精控取消与非法输入", "验证旋转取消和互斥约束"),
    CaseDefinition("RC-A-08", "自动回充成功", "验证发送、状态和结果读取"),
    CaseDefinition("RC-A-09", "自动回充取消", "验证可取消状态和取消闭环"),
    CaseDefinition("RC-A-10", "目标容差参数", "验证三个 controller 参数写后读"),
    CaseDefinition(
        "RC-A-11",
        "footprint 参数",
        "验证四点和六点 footprint 双 costmap 写后读",
    ),
    CaseDefinition("RC-A-12", "膨胀半径参数", "验证双 costmap 参数写后读"),
    CaseDefinition("RC-A-13", "跟随速度上限参数", "验证线速度和角速度上限写后读"),
    CaseDefinition("RC-A-14", "仿真模式能力边界", "验证支持项可用、缺失项 fail-closed"),
    CaseDefinition("RC-A-15", "导航非法输入拒绝", "验证错误目标不发送 Nav2 goal"),
    CaseDefinition("RC-A-16", "footprint 三点输入拒绝", "验证三点 footprint 被拒且参数不变"),
)
CASE_BY_ID = {case.case_id: case for case in CASES}
DEFAULT_REPEAT_PER_CASE = 15


@dataclass
class InterfaceCheck:
    interface: str
    expected: str
    actual: str
    normal: bool


@dataclass
class CaseResult:
    case_id: str
    name: str
    iteration: int
    status: str
    duration_sec: float
    detail: str = ""
    interfaces: list[InterfaceCheck] = field(default_factory=list)


class CaseRecorder:

    def __init__(self, definition):
        self.definition = definition
        self.checks = []

    def check(self, interface, expected, actual, normal):
        item = InterfaceCheck(
            interface=str(interface),
            expected=str(expected),
            actual=str(actual),
            normal=bool(normal),
        )
        self.checks.append(item)
        state = "NORMAL  " if item.normal else "ABNORMAL"
        print(f"  [{state}] {item.interface} | expected={item.expected} "
              f"| actual={item.actual}")
        return item.normal


class TestContext:

    def __init__(self, client, sim_client, server, parameter_nodes):
        self.client = client
        self.sim_client = sim_client
        self.server = server
        self.controller, self.local_costmap, self.global_costmap = parameter_nodes


def wait_until(predicate, timeout_sec, interval_sec=0.03):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_sec)
    return False


def _state_case(ctx, rec):
    connected = ctx.client.is_connected(timeout_sec=1.0)
    rec.check("ChassisClient.is_connected", "True", connected, connected)
    ok, pose = ctx.client.get_chassis_position()
    pose_ok = ok and pose is not None and all(
        math.isclose(float(a), b, abs_tol=0.05)
        for a, b in zip(pose, (1.0, 2.0, 45.0)))
    rec.check(
        "ChassisClient.get_chassis_position",
        "success=True, pose=[1.0, 2.0, 45.0deg]",
        f"success={ok}, pose={pose}",
        pose_ok,
    )
    battery_ready = wait_until(lambda: ctx.client.get_battery_percentage()[0],
                               2.0)
    ok, battery = ctx.client.get_battery_percentage()
    rec.check(
        "ChassisClient.get_battery_percentage",
        "success=True, battery=88.0",
        f"ready={battery_ready}, success={ok}, battery={battery}",
        battery_ready and ok and math.isclose(battery, 88.0),
    )
    obstacle = ctx.client.get_front_obstacle()
    rec.check(
        "ChassisClient.get_front_obstacle",
        "False",
        obstacle,
        obstacle is False,
    )


def _disconnect_case(ctx, rec):
    ctx.server.publish_pose_enabled = False
    try:
        disconnected = wait_until(
            lambda: not ctx.client.is_connected(timeout_sec=0.02), 1.5)
        rec.check(
            "/feedback/chassis_pose timeout",
            "disconnected within 1.5s",
            disconnected,
            disconnected,
        )
        ok, pose = ctx.client.get_chassis_position()
        rec.check(
            "stale pose invalidation",
            "success=False, pose=None",
            f"success={ok}, pose={pose}",
            not ok and pose is None,
        )
    finally:
        ctx.server.publish_pose_enabled = True
    recovered = wait_until(lambda: ctx.client.is_connected(timeout_sec=0.05),
                           1.5)
    rec.check(
        "/feedback/chassis_pose recovery",
        "connected within 1.5s",
        recovered,
        recovered,
    )


def _nav_success_case(ctx, rec):
    sent = ctx.client.set_chassis_nav2_position([1.0, 0.5, 90.0])
    rec.check("navigate_to_pose send", "True", sent, sent)
    active = wait_until(lambda: ctx.client.get_nav_state() == "NAVIGATING", 1.0)
    rec.check("get_nav_state active", "NAVIGATING", ctx.client.get_nav_state(),
              active)
    done = wait_until(lambda: ctx.client.get_nav_state() == "SUCCEEDED", 3.0)
    rec.check("get_nav_state terminal", "SUCCEEDED", ctx.client.get_nav_state(),
              done)


def _nav_cancel_case(ctx, rec):
    sent = ctx.client.set_chassis_nav2_position([2.0, 0.5, 0.0])
    rec.check("navigate_to_pose send", "True", sent, sent)
    cancelable = wait_until(ctx.client.has_cancelable_nav_goal, 1.0)
    rec.check("has_cancelable_nav_goal", "True", cancelable, cancelable)
    canceled = ctx.client.cancel_chassis_nav2()
    rec.check("cancel_chassis_nav2", "True", canceled, canceled)
    done = wait_until(lambda: ctx.client.get_nav_state() == "CANCELLED", 2.0)
    rec.check("get_nav_state terminal", "CANCELLED", ctx.client.get_nav_state(),
              done)


def _points_case(ctx, rec):
    points = ctx.client.get_all_nav_points()
    rec.check(
        "/map_asset_manager/get_all_nav_points count",
        "2",
        len(points),
        len(points) == 2,
    )
    rec.check(
        "point_a value",
        "(1.0, 0.5, 0.0)",
        points.get("point_a"),
        points.get("point_a") == (1.0, 0.5, 0.0),
    )
    point_b = points.get("point_b")
    rec.check(
        "point_b theta",
        "1.57rad",
        point_b,
        point_b is not None and
        math.isclose(point_b[2], 1.57, rel_tol=0.0, abs_tol=1e-6),
    )


def _control_success_case(ctx, rec):
    sent = ctx.client.set_chassis_control_position([0.3, 0.0, 0.0])
    rec.check("control/precise_control send", "True", sent, sent)
    active = wait_until(lambda: ctx.client.get_control_status() == "CONTROL",
                        1.0)
    rec.check("get_control_status active", "CONTROL",
              ctx.client.get_control_status(), active)
    refreshed = ctx.client.refresh_control_status()
    rec.check("refresh_control_status", "True", refreshed, refreshed is True)
    done = wait_until(lambda: ctx.client.get_control_status() == "SUCCEEDED",
                      3.0)
    rec.check("get_control_status terminal", "SUCCEEDED",
              ctx.client.get_control_status(), done)


def _control_cancel_case(ctx, rec):
    sent = ctx.client.set_chassis_control_position([0.0, 0.0, 30.0])
    rec.check("control/precise_control send", "True", sent, sent)
    cancelable = wait_until(ctx.client.has_cancelable_control_goal, 1.0)
    rec.check("has_cancelable_control_goal", "True", cancelable, cancelable)
    canceled = ctx.client.cancel_chassis_control()
    rec.check("cancel_chassis_control", "True", canceled, canceled)
    done = wait_until(lambda: ctx.client.get_control_status() == "CANCELLED",
                      2.0)
    rec.check("get_control_status terminal", "CANCELLED",
              ctx.client.get_control_status(), done)
    invalid = ctx.client.set_chassis_control_position([0.3, 0.0, 30.0])
    rec.check("x/yaw mutual exclusion", "False", invalid, not invalid)


def _charging_success_case(ctx, rec):
    sent = ctx.client.set_chassis_auto_charging(force=True, timeout_sec=1.0)
    rec.check("/auto_charging send", "True", sent, sent)
    active = wait_until(
        lambda: ctx.client.get_auto_charging_status() in
        ("ACCEPTED", "EXECUTING"),
        1.0,
    )
    rec.check(
        "get_auto_charging_status active",
        "ACCEPTED or EXECUTING",
        ctx.client.get_auto_charging_status(),
        active,
    )
    done = wait_until(
        lambda: ctx.client.get_auto_charging_status() == "SUCCEEDED", 3.0)
    rec.check(
        "get_auto_charging_status terminal",
        "SUCCEEDED",
        ctx.client.get_auto_charging_status(),
        done,
    )
    ok, message = ctx.client.get_auto_charging_result()
    rec.check(
        "get_auto_charging_result",
        "success=True, message=charging ok",
        f"success={ok}, message={message}",
        ok and message == "charging ok",
    )


def _charging_cancel_case(ctx, rec):
    sent = ctx.client.set_chassis_auto_charging(force=True, timeout_sec=1.0)
    rec.check("/auto_charging send", "True", sent, sent)
    cancelable = wait_until(ctx.client.has_cancelable_auto_charging_goal, 1.0)
    rec.check("has_cancelable_auto_charging_goal", "True", cancelable,
              cancelable)
    canceled = ctx.client.cancel_auto_charging()
    rec.check("cancel_auto_charging", "True", canceled, canceled)
    done = wait_until(
        lambda: ctx.client.get_auto_charging_status() == "CANCELED", 2.0)
    rec.check(
        "get_auto_charging_status terminal",
        "CANCELED",
        ctx.client.get_auto_charging_status(),
        done,
    )


def _goal_tolerance_case(ctx, rec):
    ok = ctx.client.set_goal_tolerance(0.1, 0.15)
    rec.check("set_goal_tolerance return", "True", ok, ok)
    for name, expected in (
        ("high_precision_goal_checker.lon_goal_tolerance", 0.1),
        ("high_precision_goal_checker.lat_goal_tolerance", 0.1),
        ("high_precision_goal_checker.yaw_goal_tolerance", 0.15),
    ):
        actual = ctx.controller.get_parameter(name).value
        rec.check(name, expected, actual, math.isclose(actual, expected))


def _footprint_case(ctx, rec):
    footprints = (
        (
            "four-point",
            [[0.3, 0.25], [0.3, -0.25], [-0.3, -0.25], [-0.3, 0.25]],
            "[[0.3,0.25],[0.3,-0.25],[-0.3,-0.25],[-0.3,0.25]]",
        ),
        (
            "six-point",
            [
                [0.35, 0.0],
                [0.2, -0.25],
                [-0.2, -0.25],
                [-0.35, 0.0],
                [-0.2, 0.25],
                [0.2, 0.25],
            ],
            "[[0.35,0.0],[0.2,-0.25],[-0.2,-0.25],[-0.35,0.0],[-0.2,0.25],[0.2,0.25]]",
        ),
    )

    for label, vertices, expected in footprints:
        ok = ctx.client.set_footprint(vertices)
        rec.check(f"{label} set_footprint return", "True", ok, ok)
        for node_name, node in (
            ("local_costmap", ctx.local_costmap),
            ("global_costmap", ctx.global_costmap),
        ):
            actual = node.get_parameter("footprint").value
            rec.check(
                f"{label} {node_name}.footprint",
                expected,
                actual,
                actual == expected,
            )


def _inflation_case(ctx, rec):
    ok = ctx.client.set_inflation_radius(0.45)
    rec.check("set_inflation_radius return", "True", ok, ok)
    for node_name, node in (
        ("local_costmap", ctx.local_costmap),
        ("global_costmap", ctx.global_costmap),
    ):
        actual = node.get_parameter("inflation_layer.inflation_radius").value
        rec.check(
            f"{node_name}.inflation_layer.inflation_radius",
            0.45,
            actual,
            math.isclose(actual, 0.45),
        )


def _velocity_case(ctx, rec):
    ok = ctx.client.set_follow_velocity_limits(0.5, 0.8)
    rec.check("set_follow_velocity_limits return", "True", ok, ok)
    for name, expected in (("FollowPath.vx_max", 0.5), ("FollowPath.wz_max",
                                                        0.8)):
        actual = ctx.controller.get_parameter(name).value
        rec.check(name, expected, actual, math.isclose(actual, expected))


def _sim_case(ctx, rec):
    connected = wait_until(lambda: ctx.sim_client.is_connected(0.05), 1.0)
    rec.check("SIM /feedback_sim/chassis_pose", "connected", connected,
              connected)
    sent = ctx.sim_client.set_chassis_control_position([0.1, 0.0, 0.0])
    rec.check("SIM control/precise_control", "True", sent, sent)
    done = wait_until(
        lambda: ctx.sim_client.get_control_status() == "SUCCEEDED", 3.0)
    rec.check(
        "SIM precise control terminal",
        "SUCCEEDED",
        ctx.sim_client.get_control_status(),
        done,
    )
    previous_checker = ctx.controller.get_parameter(
        "current_goal_checker").value
    ctx.controller.set_parameters([
        Parameter(
            "current_goal_checker",
            Parameter.Type.STRING,
            "simple_goal_checker",
        )
    ])
    try:
        tolerance_ok = ctx.sim_client.set_goal_tolerance(0.1, 0.1)
        rec.check("SIM set_goal_tolerance", "True", tolerance_ok, tolerance_ok)
        for name in (
                "simple_goal_checker.xy_goal_tolerance",
                "simple_goal_checker.yaw_goal_tolerance",
        ):
            actual = ctx.controller.get_parameter(name).value
            rec.check(name, 0.1, actual, math.isclose(actual, 0.1))
    finally:
        ctx.controller.set_parameters([
            Parameter(
                "current_goal_checker",
                Parameter.Type.STRING,
                previous_checker,
            )
        ])

    unsupported = (
        ("SIM navigate_to_pose",
         ctx.sim_client.set_chassis_nav2_position([0, 0, 0]), False),
        ("SIM get_all_nav_points", ctx.sim_client.get_all_nav_points(), {}),
        ("SIM auto_charging", ctx.sim_client.set_chassis_auto_charging(),
         False),
    )
    for name, actual, expected in unsupported:
        rec.check(name, expected, actual, actual == expected)


def _nav_invalid_input_case(ctx, rec):
    before = ctx.server.nav_goal_count
    print("  [EXPECTED_REJECTION] 发送缺少 yaw 的导航目标")
    accepted = ctx.client.set_chassis_nav2_position([1.0, 2.0])
    time.sleep(0.1)
    rec.check("invalid target return", "False", accepted, not accepted)
    rec.check(
        "navigate_to_pose goal count",
        before,
        ctx.server.nav_goal_count,
        ctx.server.nav_goal_count == before,
    )


def _footprint_three_points_case(ctx, rec):
    before_local = ctx.local_costmap.get_parameter("footprint").value
    before_global = ctx.global_costmap.get_parameter("footprint").value
    print("  [EXPECTED_REJECTION] 发送三点三角形 footprint")
    accepted = ctx.client.set_footprint([[0.3, 0.25], [0.3, -0.25], [-0.3,
                                                                     0.0]])
    rec.check("three-point footprint return", "False", accepted, not accepted)
    after_local = ctx.local_costmap.get_parameter("footprint").value
    after_global = ctx.global_costmap.get_parameter("footprint").value
    rec.check("local_costmap unchanged", before_local, after_local,
              after_local == before_local)
    rec.check("global_costmap unchanged", before_global, after_global,
              after_global == before_global)


CASE_RUNNERS: dict[str, Callable] = {
    "RC-A-01": _state_case,
    "RC-A-02": _disconnect_case,
    "RC-A-03": _nav_success_case,
    "RC-A-04": _nav_cancel_case,
    "RC-A-05": _points_case,
    "RC-A-06": _control_success_case,
    "RC-A-07": _control_cancel_case,
    "RC-A-08": _charging_success_case,
    "RC-A-09": _charging_cancel_case,
    "RC-A-10": _goal_tolerance_case,
    "RC-A-11": _footprint_case,
    "RC-A-12": _inflation_case,
    "RC-A-13": _velocity_case,
    "RC-A-14": _sim_case,
    "RC-A-15": _nav_invalid_input_case,
    "RC-A-16": _footprint_three_points_case,
}


def _package_version():
    try:
        return metadata.version("dexe_robot_client")
    except metadata.PackageNotFoundError:
        return "source-tree"


def _atomic_write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def _display_width(value):
    return sum(2 if unicodedata.east_asian_width(character) in ("W", "F") else 1
               for character in str(value))


def _print_final_summary(payload):
    headers = (
        "ID",
        "测试项",
        "计划",
        "完成",
        "PASS",
        "FAIL",
        "ERROR",
        "PENDING_ENV",
        "结论",
    )
    rows = []
    for item in payload["case_summaries"]:
        counts = item["counts"]
        rows.append((
            item["case_id"],
            item["name"],
            item["planned_runs"],
            item["completed_runs"],
            counts["PASS"],
            counts["FAIL"],
            counts["ERROR"],
            counts["PENDING_ENV"],
            item["status"],
        ))
    counts = payload["counts"]
    rows.append((
        "TOTAL",
        "全部用例",
        payload["planned_runs"],
        payload["completed_runs"],
        counts["PASS"],
        counts["FAIL"],
        counts["ERROR"],
        counts["PENDING_ENV"],
        payload["overall"],
    ))

    widths = [
        max(_display_width(row[index])
            for row in (headers, *rows))
        for index in range(len(headers))
    ]

    def render_row(row):
        cells = []
        for value, width in zip(row, widths):
            text = str(value)
            cells.append(text + " " * (width - _display_width(text)))
        return "| " + " | ".join(cells) + " |"

    separator = "+-" + "-+-".join("-" * width for width in widths) + "-+"
    print("\n自动测试最终结果")
    print(separator)
    print(render_row(headers))
    print(separator)
    for index, row in enumerate(rows):
        print(render_row(row))
        if index == len(rows) - 2:
            print(separator)
    print(separator)


def _write_results(artifact_dir, started_at, domain_id, selected, repeat_count,
                   results, setup_error):
    counts = {
        status: sum(item.status == status for item in results)
        for status in ("PASS", "FAIL", "ERROR", "PENDING_ENV")
    }
    planned_runs = len(selected) * repeat_count
    case_summaries = []
    for case_id in selected:
        definition = CASE_BY_ID[case_id]
        case_results = [item for item in results if item.case_id == case_id]
        case_counts = {
            status: sum(item.status == status for item in case_results)
            for status in ("PASS", "FAIL", "ERROR", "PENDING_ENV")
        }
        if len(case_results) != repeat_count:
            case_status = "PENDING_ENV"
        elif case_counts["FAIL"] or case_counts["ERROR"] or case_counts[
                "PENDING_ENV"]:
            case_status = "FAIL"
        else:
            case_status = "PASS"
        durations = [item.duration_sec for item in case_results]
        case_summaries.append({
            "case_id": case_id,
            "name": definition.name,
            "status": case_status,
            "planned_runs": repeat_count,
            "completed_runs": len(case_results),
            "counts": case_counts,
            "duration_sec": {
                "total":
                    round(sum(durations), 6),
                "average":
                    round(sum(durations) /
                          len(durations), 6) if durations else 0.0,
                "minimum":
                    min(durations) if durations else 0.0,
                "maximum":
                    max(durations) if durations else 0.0,
            },
        })
    if setup_error:
        overall = "PENDING_ENV"
    elif counts["FAIL"] or counts["ERROR"] or counts["PENDING_ENV"]:
        overall = "AUTOMATED_FAIL"
    elif len(results) != planned_runs:
        overall = "PENDING_ENV"
    else:
        overall = "AUTOMATED_PASS"

    payload = {
        "schema_version":
            2,
        "tool":
            "python3 -m dexe_robot_client.navigation_client.interface_test",
        "mode":
            "mock",
        "scope":
            "automatic-only",
        "started_at":
            started_at,
        "finished_at":
            datetime.now().astimezone().isoformat(timespec="seconds"),
        "overall":
            overall,
        "package_version":
            _package_version(),
        "ros_domain_id":
            domain_id,
        "selected_cases":
            selected,
        "repeat_per_case":
            repeat_count,
        "planned_runs":
            planned_runs,
        "completed_runs":
            len(results),
        "counts":
            counts,
        "case_summaries":
            case_summaries,
        "setup_error":
            setup_error,
        "tests": [asdict(item) for item in results],
    }
    _atomic_write(
        artifact_dir / "summary.json",
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
    )

    lines = [
        "# robot_client navigation_client 自动自测结果",
        "",
        f"- 总体状态：`{overall}`",
        f"- 模式：`mock`（自动测试，不代表目标机器人现场通过）",
        f"- ROS_DOMAIN_ID：`{domain_id}`",
        f"- 包版本：`{payload['package_version']}`",
        f"- 结果目录：`{artifact_dir}`",
        f"- 每个用例执行轮次：`{repeat_count}`",
        f"- 计划/完成调用：`{planned_runs}/{len(results)}`",
        "",
        "| ID | 测试项 | 通过轮次 | 状态 | 平均耗时(s) | 最小(s) | 最大(s) |",
        "|---|---|---:|---|---:|---:|---:|",
    ]
    for item in case_summaries:
        duration = item["duration_sec"]
        lines.append(
            f"| {item['case_id']} | {item['name']} "
            f"| {item['counts']['PASS']}/{item['planned_runs']} | {item['status']} "
            f"| {duration['average']:.3f} | {duration['minimum']:.3f} "
            f"| {duration['maximum']:.3f} |")
    if setup_error:
        lines.extend(["", "## 环境阻塞", "", setup_error])
    for result in results:
        lines.extend([
            "",
            f"## {result.case_id}/R{result.iteration:02d} {result.name}",
            "",
        ])
        if result.detail:
            lines.extend([result.detail, ""])
        lines.extend([
            "| 接口/检查点 | 期望 | 实际 | 判定 |",
            "|---|---|---|---|",
        ])
        for check in result.interfaces:
            verdict = "NORMAL" if check.normal else "ABNORMAL"
            lines.append(
                f"| {check.interface} | {check.expected} | {check.actual} | {verdict} |"
            )
    _atomic_write(artifact_dir / "summary.md", "\n".join(lines) + "\n")

    flow = [
        "# 自动测试执行流程",
        "",
        f"- 启动时间：`{started_at}`",
        f"- 隔离域：`ROS_DOMAIN_ID={domain_id}`",
        "- 步骤 1：启动进程内 mock topic/action/service/parameter peers。",
        "- 步骤 2：启动 real/sim 两个 ChassisClient 并交给同一多线程 executor。",
        f"- 步骤 3：按编号顺序执行选定 RC-A 用例，每项连续 {repeat_count} 轮；单轮异常不删除、不伪装为 PASS。",
        "- 步骤 4：测试循环结束后释放节点、executor 与 rclpy。",
        "- 步骤 5：写入 summary.json、summary.md 与 test_flow.md，再从同一结果数据输出终端最终表格。",
        "",
        "## 实际顺序",
        "",
    ]
    for index, result in enumerate(results, start=1):
        flow.append(f"{index}. `{result.case_id}/R{result.iteration:02d}` "
                    f"{result.name} → `{result.status}`")
    _atomic_write(artifact_dir / "test_flow.md", "\n".join(flow) + "\n")
    return payload


def run_mock(args):
    domain_id = args.domain_id
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    artifact_dir = (args.artifact_dir.expanduser().resolve()
                    if args.artifact_dir else Path.home() /
                    ".dexe/navigation/robot_client_interface_test" /
                    f"mock_{timestamp}")
    artifact_dir.mkdir(parents=True, exist_ok=False)
    selected = args.case or [case.case_id for case in CASES]
    results = []
    setup_error = ""
    executor = None
    spin_thread = None
    nodes = []
    rclpy_started = False

    try:
        rclpy.init()
        rclpy_started = True
        server = MockChassisServer()
        parameter_nodes = make_parameter_nodes()
        client = ChassisClient(is_sim=False, timeout_sec=0.35)
        sim_client = ChassisClient(is_sim=True, timeout_sec=0.35)
        nodes = [server, *parameter_nodes, client, sim_client]
        executor = MultiThreadedExecutor(num_threads=10)
        for node in nodes:
            executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        context = TestContext(client, sim_client, server, parameter_nodes)

        ready = wait_until(lambda: client.is_connected(0.05), 2.0)
        if not ready:
            raise RuntimeError("2 秒内未建立 mock 位姿连接")

        for case_id in selected:
            definition = CASE_BY_ID[case_id]
            for iteration in range(1, args.repeat + 1):
                recorder = CaseRecorder(definition)
                run_id = f"{case_id}/R{iteration:02d}"
                print(f"\n[TEST] {run_id} {definition.name}")
                start = time.monotonic()
                detail = ""
                status = "ERROR"
                try:
                    CASE_RUNNERS[case_id](context, recorder)
                    status = ("PASS" if recorder.checks and all(
                        item.normal for item in recorder.checks) else "FAIL")
                except Exception as error:  # noqa: BLE001
                    detail = f"{type(error).__name__}: {error}"
                    status = "ERROR"
                duration = time.monotonic() - start
                results.append(
                    CaseResult(
                        case_id=case_id,
                        name=definition.name,
                        iteration=iteration,
                        status=status,
                        duration_sec=round(duration, 6),
                        detail=detail,
                        interfaces=recorder.checks,
                    ))
                print(f"[{status:<11}] {run_id}: {definition.name}")
    except Exception as error:  # noqa: BLE001
        setup_error = f"{type(error).__name__}: {error}"
        print(f"[PENDING_ENV] {setup_error}")
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        for node in reversed(nodes):
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if rclpy_started:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass

    payload = _write_results(
        artifact_dir,
        started_at,
        domain_id,
        selected,
        args.repeat,
        results,
        setup_error,
    )
    _print_final_summary(payload)
    overall = payload["overall"]
    print(f"\n结果：{overall}")
    print(f"产物：{artifact_dir}")
    if overall == "AUTOMATED_PASS":
        return 0
    if overall == "PENDING_ENV":
        return 2
    return 1


def list_tests():
    print("automatic cases (manual cases are intentionally excluded; "
          f"formal default={DEFAULT_REPEAT_PER_CASE} runs/case):")
    for case in CASES:
        print(f"{case.case_id}\t{case.name}\t{case.purpose}")
    return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="robot_client navigation_client 自动接口自测（不含人工现场项）")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list-tests", help="列出自动用例目录")
    mock = subparsers.add_parser("mock",
                                 help="在隔离 ROS domain 启动进程内 mock 并执行自动用例")
    mock.add_argument(
        "--case",
        action="append",
        choices=tuple(CASE_BY_ID),
        help="只执行指定用例；可重复，默认执行全部",
    )
    mock.add_argument(
        "--domain-id",
        type=int,
        default=188,
        choices=range(0, 233),
        metavar="0..232",
        help="隔离 ROS_DOMAIN_ID，默认 188",
    )
    mock.add_argument("--artifact-dir", type=Path, help="指定本轮新结果目录；目录必须不存在")
    mock.add_argument(
        "--repeat",
        type=int,
        default=DEFAULT_REPEAT_PER_CASE,
        choices=range(DEFAULT_REPEAT_PER_CASE, 101),
        metavar="15..100",
        help=f"每个用例执行轮次，默认且最少 {DEFAULT_REPEAT_PER_CASE}",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.command == "list-tests":
        return list_tests()
    if args.command == "mock":
        return run_mock(args)
    return 2


if __name__ == "__main__":
    sys.exit(main())
