import json
import math
import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from cloudia_chassis_adapter.protocol import (build_speed_command,
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
        self.assertEqual(parsed["pose"]["x"], 1.0)
        self.assertEqual(len(parsed["batteries"]), 2)
        self.assertEqual(parsed["batteries"][0]["location"], "battery1")
        self.assertTrue(math.isclose(parsed["batteries"][0]["percentage"],
                                     0.42))
        self.assertEqual(parsed["batteries"][0]["cell_voltage"], [3.8, 3.9])
        self.assertEqual(parsed["batteries"][1]["location"], "battery2")
        self.assertTrue(math.isclose(parsed["batteries"][1]["percentage"],
                                     0.73))
        self.assertTrue(parsed["obstacle_stop"])

    def test_speed_envelope(self):
        payload = build_speed_command(0.1, -0.2, 0.3, use_safe=True)
        self.assertEqual(payload["communicationType"], "downstream_command")
        self.assertEqual(payload["speedCommand"]["targetVelocity"]["vy"], -0.2)
        self.assertTrue(payload["speedCommand"]["useSafe"])
        json.dumps(payload)


if __name__ == "__main__":
    unittest.main()
