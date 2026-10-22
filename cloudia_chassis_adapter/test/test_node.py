"""Exercise real adapter methods with in-memory publishers; no ROS graph or robot connection."""

import json
import math
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from cloudia_chassis_adapter.node import CloudiaChassisAdapter


class FrameTest(unittest.TestCase):

    def setUp(self):
        self.adapter = SimpleNamespace(
            _enabled=True, _raw_state_pub=Mock(), _pose_pub=Mock(),
            _frame_id="map", _last_state_time=None, _stale_reported=True,
            _lock=threading.RLock(), _connected=True,
            _stop=threading.Event(), _stop_frames_remaining=3,
            _last_cmd=(0.1, -0.2, 0.3), _last_cmd_time=9.9,
            _command_timeout=0.5, _stale_timeout=0.5,
            _send_command=Mock(return_value=True), get_logger=Mock(return_value=Mock()),
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123))))
        self.adapter._publish_pose = lambda pose: CloudiaChassisAdapter._publish_pose(self.adapter, pose)

    def handle(self, raw):
        with patch("cloudia_chassis_adapter.node.time.monotonic", return_value=10.0):
            CloudiaChassisAdapter._handle_frame(self.adapter, raw)

    def test_raw_text_and_bytes_preserve_all_fields(self):
        raw = '''{ "communicationType": "upstream_state", "header": {"version":"v9"},
 "state": {"agvPosition":{"x":1.0,"y":-2.0,"theta":0.5},
 "batteryState":{"percentage":73},"battery2State":{"percentage":0.42},
 "velocity":{"vx":0.2}, "新增字段":[null,1,{"x":true}]},
 "information":{"obstacleStop":true,"emergency":false}, "unknown": "保持空格" }'''
        for frame in (raw, raw.encode("utf-8")):
            with self.subTest(binary=isinstance(frame, bytes)):
                self.handle(frame)
                self.assertEqual(self.adapter._raw_state_pub.publish.call_args.args[0].data, raw)
                self.assertEqual(self.adapter._last_state_time, 10.0)
                self.assertFalse(self.adapter._stale_reported)
                pose = self.adapter._pose_pub.publish.call_args.args[0]
                self.assertEqual(pose.header.frame_id, "map")
                self.assertEqual(pose.header.stamp.sec, 123)
                self.assertEqual(pose.pose.position.x, 1.0)
                self.assertEqual(pose.pose.position.y, -2.0)
                self.assertAlmostEqual(pose.pose.orientation.z, math.sin(0.25))
                self.assertAlmostEqual(pose.pose.orientation.w, math.cos(0.25))

    def test_other_state_forwarded_without_pose(self):
        for state in ({"batteryState": {"percentage": 0.5}},
                      {"agvPosition": [1, 2]},
                      {"agvPosition": {"x": "NaN", "y": 0, "theta": 0}}):
            self.handle(json.dumps({"communicationType": "upstream_state", "state": state}))
        self.assertEqual(self.adapter._raw_state_pub.publish.call_count, 3)
        self.adapter._pose_pub.publish.assert_not_called()
        self.assertEqual(self.adapter._last_state_time, 10.0)

    def test_invalid_state_forwarded_but_not_fresh(self):
        for state in (None, {}, [], "invalid"):
            raw = json.dumps({"communicationType": "upstream_state", "state": state})
            self.handle(raw)
            self.assertEqual(self.adapter._raw_state_pub.publish.call_args.args[0].data, raw)
        self.assertIsNone(self.adapter._last_state_time)
        self.adapter._pose_pub.publish.assert_not_called()

    def test_non_state_or_invalid_frames_ignored(self):
        for raw in (None, b"\xff", "broken", "[]", "null", "{}",
                    '{"communicationType":"downstream_command"}'):
            self.handle(raw)
        self.adapter._raw_state_pub.publish.assert_not_called()
        self.adapter._pose_pub.publish.assert_not_called()
        self.assertIsNone(self.adapter._last_state_time)

    def test_disabled_backend_does_not_publish(self):
        self.adapter._enabled = False
        self.handle('{"communicationType":"upstream_state","state":{"batteryState":{}}}')
        self.adapter._raw_state_pub.publish.assert_not_called()
        self.assertIsNone(self.adapter._last_state_time)

    def test_command_and_state_timeouts_keep_zero_protection(self):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = 0.1, -0.2, 0.3
        with patch("cloudia_chassis_adapter.node.time.monotonic", return_value=10.0):
            CloudiaChassisAdapter._on_cmd_vel(self.adapter, msg)
            self.adapter._last_state_time = 9.9
            CloudiaChassisAdapter._send_current_command(self.adapter)
            self.adapter._send_command.assert_called_with(0.1, -0.2, 0.3)
            self.adapter._last_state_time = 9.0
            CloudiaChassisAdapter._send_current_command(self.adapter)
            self.adapter._send_command.assert_called_with(0.0, 0.0, 0.0)
            self.adapter._last_state_time = 9.9
            self.adapter._last_cmd_time = 9.0
            CloudiaChassisAdapter._send_current_command(self.adapter)
            self.adapter._send_command.assert_called_with(0.0, 0.0, 0.0)
            self.adapter._connected = False
            self.adapter._send_command.reset_mock()
            CloudiaChassisAdapter._send_current_command(self.adapter)
            self.adapter._send_command.assert_not_called()

    def test_watchdog_does_not_republish_cached_state(self):
        self.adapter._last_state_time = 9.0
        self.adapter._stale_reported = False
        with patch("cloudia_chassis_adapter.node.time.monotonic", return_value=10.0):
            CloudiaChassisAdapter._watchdog(self.adapter)
        self.assertTrue(self.adapter._stale_reported)
        self.adapter._raw_state_pub.publish.assert_not_called()
        self.adapter._pose_pub.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
