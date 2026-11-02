"""迦智后端多点导航：交管点列下发（编号解析）与坐标多点（首点补齐、连通校验）。

用 test_support 的 mock 网关跑真实的 HTTP、任务和点位模块，不起 ROS 节点。
"""

import logging
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("requests")

from dexe_robot_client.navigation_client.common import TaskState
from dexe_robot_client.navigation_client.features.cloudia import (
    CloudiaHttpClient, CloudiaNavigationFeature, CloudiaNavPointsFeature,
    CloudiaTaskClient)
from dexe_robot_client.navigation_client.test_support.mock_cloudia import \
    MockCloudiaServer


class _Node:

    def get_logger(self):
        return logging.getLogger("test_cloudia_graph_points")


@pytest.fixture
def stack():
    server = MockCloudiaServer(token="test-token", finish_after_polls=2).start()
    cfg = SimpleNamespace(
        cloudia_host=server.host,
        cloudia_task_port=server.task_port,
        cloudia_token="test-token",
        cloudia_poll_interval_s=0.05,
        cloudia_poll_fail_limit=5,
        cloudia_snap_xy_tol_m=0.3,
        cloudia_snap_yaw_tol_deg=15.0,
        cloudia_edges_bidirectional=True,
    )
    node = _Node()
    api = CloudiaHttpClient(node, cfg)
    sent = []
    start_task = api.start_task

    def capture(task_type, **fields):
        sent.append((task_type, fields))
        return start_task(task_type, **fields)

    api.start_task = capture
    task = CloudiaTaskClient(node, api, cfg)
    points = CloudiaNavPointsFeature(node, api)
    # mock 网关的车停在 1 号点 (0, 0)。
    nav = CloudiaNavigationFeature(node, task, points, lambda:
                                   (True, [0.0, 0.0, 0.0]), cfg)
    yield SimpleNamespace(nav=nav, task=task, sent=sent, cfg=cfg)
    task.close()
    server.stop()


def _wait_terminal(task, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if task.state in CloudiaTaskClient._TERMINAL:
            return task.state
        time.sleep(0.02)
    return task.state


def test_traffic_route_sent_as_given(stack):
    # 交管点列首点就是车当前所在点，原样下发。
    assert stack.nav.send_graph_points(["1", "2"])
    assert stack.sent == [("navigation", {"pointIds": [1, 2]})]
    assert _wait_terminal(stack.task) == TaskState.SUCCEEDED
    assert stack.nav.status() == "SUCCEEDED"


def test_single_point_uses_point_id(stack):
    assert stack.nav.send_graph_points(["point_2"])
    assert stack.sent == [("navigation", {"pointId": 2})]


def test_scheduler_station_number_resolves(stack):
    # 交管下发：id 是调度路网站点编号 S<点号>，name 是迦智点位名。
    assert stack.nav.send_graph_points(["站点A", "站点B"],
                                       nav_point_ids=["S001", "S002"])
    assert stack.sent == [("navigation", {"pointIds": [1, 2]})]
    assert _wait_terminal(stack.task) == TaskState.SUCCEEDED

    # 编号优先：名称恰好是另一个点号时，以编号为准。
    assert stack.nav.send_graph_points(["2", "1"],
                                       nav_point_ids=["S001", "S002"])
    assert stack.sent[-1] == ("navigation", {"pointIds": [1, 2]})


def test_rejects_without_sending(stack):
    assert not stack.nav.send_graph_points([])
    assert not stack.nav.send_graph_points(["99"])
    assert not stack.nav.send_graph_points(["1", "2"], nav_point_ids=["S001"])
    # L / M 等其他类型的调度节点不对应迦智点位。
    assert not stack.nav.send_graph_points(["站点B"], nav_point_ids=["L002"])
    assert not stack.nav.send_graph_points(["S"])
    assert stack.sent == []


def test_road_prepends_current_point(stack):
    # 坐标多点：调用方不知道迦智点位，由客户端补当前点为首点。
    assert stack.nav.send_road([[1.0, 0.0, 0.0]])
    assert stack.sent == [("navigation", {"pointIds": [1, 2]})]


def test_road_rejects_disconnected_edge(stack):
    stack.nav._edges_bidirectional = False
    # 路网只有 1 -> 2，没有 2 -> 1。
    assert not stack.nav.send_road([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    assert stack.sent == []
