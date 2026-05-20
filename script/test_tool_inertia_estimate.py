#!/usr/bin/env python3

import unittest

import numpy as np

import tool_inertia_estimate as tie


class ToolInertiaEstimateTest(unittest.TestCase):
    def test_recommended_wrist_excitation_modifies_only_wrist_and_returns_to_center(self):
        base = np.arange(20, dtype=float) * 0.01
        amplitudes = np.array([0.20, 0.18, 0.25])
        frequencies = np.array([0.35, 0.50, 0.70])
        phases = np.array([0.0, np.pi / 2.0, np.pi / 3.0])

        start_target = tie.generate_wrist_excitation_target(
            base, "primary", 0.0, 30.0, amplitudes, frequencies, phases, 2.0
        )
        mid_target = tie.generate_wrist_excitation_target(
            base, "primary", 8.0, 30.0, amplitudes, frequencies, phases, 2.0
        )
        end_target = tie.generate_wrist_excitation_target(
            base, "primary", 30.0, 30.0, amplitudes, frequencies, phases, 2.0
        )

        np.testing.assert_allclose(start_target, base)
        np.testing.assert_allclose(end_target, base)
        changed = np.flatnonzero(np.abs(mid_target - base) > 1e-12).tolist()
        self.assertEqual(changed, [10, 11, 12])

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
