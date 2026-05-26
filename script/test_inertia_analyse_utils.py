#!/usr/bin/env python3
import unittest

from inertia_analyse_utils import extract_force_sample


class InertiaAnalyseUtilsTest(unittest.TestCase):
    def test_primary_reconstructs_force_before_inertia(self):
        data = {
            "timestamp": 123,
            "raw_force_sensor": list(range(100, 112)),
            "force_sensor": list(range(10, 22)),
            "force_sensor_inertia": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        }

        sample = extract_force_sample(data, "primary", 1.25)

        self.assertEqual(sample["raw_force_sensor"], [100, 101, 102, 103, 104, 105])
        self.assertEqual(sample["force_after_inertia"], [10, 11, 12, 13, 14, 15])
        self.assertEqual(
            sample["force_after_gravity_before_inertia"],
            [10.1, 11.2, 12.3, 13.4, 14.5, 15.6],
        )


if __name__ == "__main__":
    unittest.main()
