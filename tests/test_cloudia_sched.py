"""迦智调度导航（CloudiaSchedNavigationFeature）测试：假调度 + 真 HTTP，无需 ROS 节点。"""

import time
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")

from dexe_robot_client.navigation_client.features.cloudia.scheduler import (  # noqa: E402
    CloudiaSchedHttpClient, CloudiaSchedNavigationFeature)
from dexe_robot_client.navigation_client.test_support.mock_cloudia_sched import (  # noqa: E402
    MockCloudiaSchedServer)


class _Logger:

    def __getattr__(self, _name):
        return lambda *_a, **_k: None


class _Node:

    def get_logger(self):
        return _Logger()


def _cfg(server, **overrides):
    values = dict(cloudia_sched_url=server.url,
                  cloudia_sched_robot=1,
                  cloudia_sched_map=None,
                  cloudia_poll_interval_s=0.02,
                  cloudia_poll_fail_limit=3)
    values.update(overrides)
    return SimpleNamespace(**values)


@pytest.fixture
def server():
    srv = MockCloudiaSchedServer().start()
    yield srv
    srv.stop()


def _make(server, **overrides):
    cfg = _cfg(server, **overrides)
    return CloudiaSchedNavigationFeature(
        _Node(), CloudiaSchedHttpClient(cfg.cloudia_sched_url), cfg)


def _wait(feature, expected, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if feature.state() == expected:
            return True
        time.sleep(0.02)
    return False


def test_send_id_uses_target_and_robot_map(server):
    nav = _make(server)
    assert nav.send(2)
    step = server.created[0]["steps"][0]
    assert step == {"map_name": "m1", "target": 2, "action": ""}
    assert server.created[0]["req_robot"] == 1
    assert _wait(nav, "SUCCEEDED")
    assert nav.result() == (True, "finished")
    nav.close()


def test_send_code_uses_target_code_and_config_map(server):
    nav = _make(server, cloudia_sched_map="cfgmap")
    assert nav.send("A")
    step = server.created[0]["steps"][0]
    assert step == {"map_name": "cfgmap", "target_code": "A", "action": ""}
    nav.close()


@pytest.mark.parametrize("bad", [True, 0, -1, "", "  ", [1, 2, 3], None])
def test_invalid_target_rejected_without_request(server, bad):
    nav = _make(server)
    assert not nav.send(bad)
    assert server.created == []
    assert nav.state() == "FAILED"


def test_send_with_charge_action(server):
    nav = _make(server)
    assert nav.send(3, action="charge")
    assert server.created[0]["steps"] == [{
        "map_name": "m1",
        "target": 3,
        "action": "charge"
    }]
    nav.close()


def test_stop_charge_is_in_place_action(server):
    nav = _make(server)
    assert nav.send(0, action="stopCharge")
    assert server.created[0]["steps"] == [{
        "map_name": "m1",
        "target": 0,
        "action": "stopCharge"
    }]
    assert _wait(nav, "SUCCEEDED")
    nav.close()


def test_failed_mission_reports_err_msg(server):
    server.final_state = 7
    nav = _make(server)
    assert nav.send(2)
    assert _wait(nav, "FAILED")
    assert nav.result() == (False, "mock failure")
    nav.close()


def test_cancel_reports_cancelled_not_failed(server):
    server.finish_after_polls = 1000
    nav = _make(server)
    assert nav.send(2)
    assert nav.has_cancelable_goal()
    assert nav.cancel()
    assert _wait(nav, "CANCELLED")
    assert not nav.has_cancelable_goal()
    nav.close()


def test_concurrent_send_rejected(server):
    server.finish_after_polls = 1000
    nav = _make(server)
    assert nav.send(2)
    assert not nav.send(3)
    assert len(server.created) == 1
    nav.close()


def test_poll_failures_become_lost(server):
    server.finish_after_polls = 1000
    nav = _make(server)
    assert nav.send(2)
    server.stop()
    assert _wait(nav, "FAILED")
    assert nav.result()[0] is False
    nav.close()


def test_unreachable_scheduler_fails_to_send():
    cfg = SimpleNamespace(cloudia_sched_url="http://127.0.0.1:1",
                          cloudia_sched_robot=1,
                          cloudia_sched_map="m1",
                          cloudia_poll_interval_s=0.02,
                          cloudia_poll_fail_limit=3)
    nav = CloudiaSchedNavigationFeature(
        _Node(), CloudiaSchedHttpClient(cfg.cloudia_sched_url), cfg)
    assert not nav.send(2)
    assert nav.state() == "FAILED"


def test_cancel_soft_stops_before_cancel(server):
    server.finish_after_polls = 1000
    nav = _make(server)
    assert nav.send(2)
    assert server.robcmds == []  # 没打过软急停就不清
    assert nav.cancel()
    assert server.calls == [
        "/api/v1/missions", "/api/v1/robcmd", "/api/v1/mscmds"
    ]
    assert server.robcmds == [{"robot_id": 1, "cmd": 1}]
    assert _wait(nav, "CANCELLED")
    nav.close()


def test_send_after_cancel_clears_soft_stop_first(server):
    server.finish_after_polls = 1000
    nav = _make(server)
    assert nav.send(2)
    assert nav.cancel()
    assert _wait(nav, "CANCELLED")
    assert nav.send(3)
    assert server.calls[-2:] == ["/api/v1/robcmd", "/api/v1/missions"]
    assert server.robcmds[-1] == {"robot_id": 1, "cmd": 2}
    nav.close()


def test_soft_stop_failure_still_cancels(server):
    server.finish_after_polls = 1000
    server.robcmd_code = 1
    nav = _make(server)
    assert nav.send(2)
    assert nav.cancel()
    assert _wait(nav, "CANCELLED")
    # 软急停请求失败也可能已生效（如超时），下次下发照样先清
    server.robcmd_code = 0
    assert nav.send(3)
    assert server.robcmds[-1] == {"robot_id": 1, "cmd": 2}
    nav.close()


def test_clear_soft_stop_failure_blocks_send_and_retries(server):
    server.finish_after_polls = 1000
    nav = _make(server)
    assert nav.send(2)
    assert nav.cancel()
    assert _wait(nav, "CANCELLED")
    server.robcmd_code = 1
    assert not nav.send(3)
    assert len(server.created) == 1  # 没清掉就不下发
    assert nav.state() == "FAILED"
    server.robcmd_code = 0
    assert nav.send(3)
    assert server.robcmds[-1] == {"robot_id": 1, "cmd": 2}
    assert len(server.created) == 2
    nav.close()
