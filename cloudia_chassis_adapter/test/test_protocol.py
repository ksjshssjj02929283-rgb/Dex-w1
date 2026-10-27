import json
import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from cloudia_chassis_adapter.protocol import (build_heartbeat_command,
                                              build_speed_command,
                                              is_heartbeat_response,
                                              parse_upstream_state)


class ProtocolTest(unittest.TestCase):

    def test_parse_state(self):
        parsed = parse_upstream_state({
            "communicationType": "upstream_state",
            "information": {
                "obstacleStop": True
            },
            "state": {
                "agvPosition": {
                    "x": 1.0,
                    "y": -2.0,
                    "theta": 0.5
                },
                "velocity": {
                    "vx": 0.2,
                    "vy": 0.1,
                    "omega": -0.3
                },
                "batteryState": {
                    "percentage": 0.42,
                    "voltage": 48.5,
                    "current": -1.2,
                    "charge": 32.0,
                    "capacity": 44.0,
                    "designCapacity": 50.0,
                    "temperature": 24.0,
                    "powerSupplyStatus": 2,
                    "powerSupplyHealth": 1,
                    "powerSupplyTechnology": 3,
                    "present": True,
                    "cellVoltages": [3.8, 3.9],
                },
                "battery2State": {
                    "percentage": 73.0,
                    "present": True
                },
            },
        })
        self.assertEqual(parsed, {"pose": {"x": 1.0, "y": -2.0, "theta": 0.5}})

    def test_invalid_state_is_not_heartbeat(self):
        for state in (None, {}, [], False, "invalid", 42):
            with self.subTest(state=state):
                self.assertIsNone(
                    parse_upstream_state({
                        "communicationType": "upstream_state",
                        "state": state
                    }))
        for payload in (None, [], {}, {"communicationType": "reply"}):
            with self.subTest(payload=payload):
                self.assertIsNone(parse_upstream_state(payload))

    def test_invalid_pose_does_not_reject_other_state(self):
        for position in (None, {}, [], "bad", {
                "x": 1
        }, {
                "x": float("nan"),
                "y": 0,
                "theta": 0
        }, {
                "x": 0,
                "y": 0,
                "theta": "inf"
        }, {
                "x": 10**1000,
                "y": 0,
                "theta": 0
        }):
            with self.subTest(position=position):
                self.assertEqual(
                    parse_upstream_state({
                        "communicationType": "upstream_state",
                        "state": {
                            "agvPosition": position,
                            "batteryState": {}
                        }
                    }), {"pose": None})

    def test_speed_envelope(self):
        payload = build_speed_command(0.1, -0.2, 0.3, use_safe=True)
        self.assertEqual(payload["communicationType"], "downstream_command")
        self.assertEqual(payload["speedCommand"]["targetVelocity"]["vy"], -0.2)
        self.assertTrue(payload["speedCommand"]["useSafe"])
        json.dumps(payload)


class HeartbeatProtocolTest(unittest.TestCase):

    def test_heartbeat_command_shape(self):
        payload = build_heartbeat_command("robot-a", serial_number="SN1")
        self.assertEqual(payload["communicationType"], "downstream_command")
        self.assertEqual(payload["heartbeatCommand"], {"clientId": "robot-a"})
        self.assertEqual(payload["header"]["serialNumber"], "SN1")
        self.assertNotIn("speedCommand", payload)
        json.dumps(payload)

    def test_heartbeat_response_detection(self):
        self.assertTrue(is_heartbeat_response({"type": "heartbeatResponse"}))
        for payload in ({"communicationType": "upstream_state"}, {}, [], None,
                        {"type": "other"}):
            self.assertFalse(is_heartbeat_response(payload))


if __name__ == "__main__":
    unittest.main()
