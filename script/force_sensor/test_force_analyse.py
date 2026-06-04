import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("force_analyse.py")
SPEC = importlib.util.spec_from_file_location("force_analyse", MODULE_PATH)
force_analyse = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(force_analyse)


class ForceAnalyseContractTest(unittest.TestCase):
    def test_payload_metadata_only_lists_core_fields(self):
        sample = {
            "t": 0.0,
            "timestamp": 1.23,
            "force_sensor": [0.0] * 6,
            "force_sensor_gravity": [0.0] * 6,
            "raw_force_sensor": [0.0] * 6,
            "force_sensor_bias": [0.0] * 6,
            "force_sensor_inertia": [0.0] * 6,
            "ee_omega_ee_frame": [0.0] * 3,
            "ee_alpha_ee_frame": [0.0] * 3,
        }

        payload = force_analyse.make_payload([sample], state_topic="/feedback/robot_server_state")

        self.assertEqual(
            payload["metadata"]["fields"],
            [
                "force_sensor",
                "force_sensor_gravity",
                "raw_force_sensor",
                "force_sensor_bias",
                "force_sensor_inertia",
                "ee_omega_ee_frame",
                "ee_alpha_ee_frame",
                "joint_position",
                "joint_velocity",
            ],
        )


if __name__ == "__main__":
    unittest.main()
