#!/usr/bin/env python3

import unittest

import numpy as np

import tool_inertia_estimate as tie


class ToolInertiaEstimateTest(unittest.TestCase):
    def test_fit_inertia_from_synthetic_rotational_wrench(self):
        rng = np.random.default_rng(42)
        true_theta = np.array([0.018, 0.026, 0.031, 0.002, -0.003, 0.004])
        true_inertia = tie.theta_to_inertia_matrix(true_theta)

        omega = rng.normal(size=(240, 3))
        alpha = rng.normal(size=(240, 3))
        torque = np.vstack(
            [
                true_inertia @ alpha[i]
                + np.cross(omega[i], true_inertia @ omega[i])
                for i in range(omega.shape[0])
            ]
        )

        result = tie.fit_inertia_from_rotational_terms(omega, alpha, torque)

        np.testing.assert_allclose(result["theta"], true_theta, atol=1e-10)
        np.testing.assert_allclose(result["inertia_com"], true_inertia, atol=1e-10)
        self.assertLess(result["torque_rms"], 1e-10)
        self.assertTrue(result["physical"]["symmetric"])
        self.assertTrue(result["physical"]["positive_definite"])


if __name__ == "__main__":
    unittest.main()
