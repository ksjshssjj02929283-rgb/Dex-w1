#!/usr/bin/env python3
"""Unit checks for force_analyse derived fields."""

import unittest

import force_analyse


class ForceAnalyseTest(unittest.TestCase):
    def test_extract_force_sample_derives_inertia_as_force_minus_gravity(self):
        data = {
            "timestamp": 123,
            "force_sensor": [10.0, -2.0, 3.0, 0.5, -0.6, 0.7, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            "force_sensor_gravity": [8.0, -5.0, 1.5, 0.2, -0.1, 0.9, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5],
            "raw_force_sensor": [0.0] * 12,
            "force_sensor_bias": [0.0] * 12,
        }

        sample = force_analyse.extract_force_sample(data, t=1.25)

        self.assertEqual(
            sample["force_sensor_inertia"],
            [2.0, 3.0, 1.5, 0.3, -0.5, -0.2, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        )

    def test_field_matrix_derives_inertia_for_older_recordings(self):
        sample = {
            "force_sensor": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            "force_sensor_gravity": [0.5, 1.5, 4.0, 3.0, 5.5, 7.5],
        }

        matrix = force_analyse.field_matrix([sample], "force_sensor_inertia", "primary")

        self.assertEqual(matrix, [[0.5, 0.5, -1.0, 1.0, -0.5, -1.5]])


if __name__ == "__main__":
    unittest.main()
