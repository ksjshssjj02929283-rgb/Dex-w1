import sys
import unittest
from pathlib import Path

import numpy as np
from numpy.testing import assert_allclose
from scipy import signal

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dyn_params_ident.acceleration_free_regressor import (
    build_filtered_regressor,
    lowpass_filter,
    resample_recordings,
)


def reference_filter(time, values, rate):
    """Use an independent continuous-time LTI solver as the filter oracle."""
    values = np.asarray(values)
    flat = values.reshape(len(time), -1)
    output = np.column_stack(
        [
            signal.lsim(([rate], [1.0, rate]), flat[:, i], time - time[0])[1]
            for i in range(flat.shape[1])
        ]
    )
    return output.reshape(values.shape)


def single_joint_regressor(q, dq, ddq):
    # T = (a + b*sin(q))*dq**2/2, V = -g*cos(q).
    angle, velocity, acceleration = q[0], dq[0], ddq[0]
    return np.array(
        [[
            acceleration,
            np.sin(angle) * acceleration + 0.5 * np.cos(angle) * velocity**2,
            np.sin(angle),
            velocity,
            np.sign(velocity),
            1.0,
        ]]
    )


def single_joint_recording(time, phase=0.0):
    a1 = 0.83 * time + phase
    a2 = 1.61 * time + 0.2 - 0.4 * phase
    q = (0.3 + 0.55 * np.sin(a1) + 0.2 * np.sin(a2))[:, None]
    dq = (0.55 * 0.83 * np.cos(a1) + 0.2 * 1.61 * np.cos(a2))[:, None]
    ddq = (-0.55 * 0.83**2 * np.sin(a1) - 0.2 * 1.61**2 * np.sin(a2))[:, None]
    parameters = np.array([2.8, 0.65, 8.0, 0.22, 0.34, -0.09])
    # The torque is written directly from Euler-Lagrange dynamics, independently
    # of the callback passed to the acceleration-free implementation.
    tau = (
        (parameters[0] + parameters[1] * np.sin(q)) * ddq
        + 0.5 * parameters[1] * np.cos(q) * dq**2
        + parameters[2] * np.sin(q)
        + parameters[3] * dq
        + parameters[4] * np.sign(dq)
        + parameters[5]
    )
    return q, dq, tau, parameters


def coupled_regressor(q, dq, ddq):
    # T = (a*dq1**2 + b*dq2**2)/2 + c*cos(q2)*dq1*dq2.
    return np.array(
        [
            [ddq[0], 0.0, np.cos(q[1]) * ddq[1] - np.sin(q[1]) * dq[1] ** 2,
             np.sin(q[0]), 0.0],
            [0.0, ddq[1], np.cos(q[1]) * ddq[0], 0.0, np.sin(q[1])],
        ]
    )


def coupled_recording(time):
    q = np.column_stack(
        (
            0.35 + 0.4 * np.sin(0.81 * time) + 0.15 * np.sin(1.9 * time + 0.2),
            -0.28 + 0.6 * np.sin(1.13 * time + 0.4) + 0.13 * np.cos(2.2 * time),
        )
    )
    dq = np.column_stack(
        (
            0.4 * 0.81 * np.cos(0.81 * time) + 0.15 * 1.9 * np.cos(1.9 * time + 0.2),
            0.6 * 1.13 * np.cos(1.13 * time + 0.4) - 0.13 * 2.2 * np.sin(2.2 * time),
        )
    )
    ddq = np.column_stack(
        (
            -0.4 * 0.81**2 * np.sin(0.81 * time) - 0.15 * 1.9**2 * np.sin(1.9 * time + 0.2),
            -0.6 * 1.13**2 * np.sin(1.13 * time + 0.4) - 0.13 * 2.2**2 * np.cos(2.2 * time),
        )
    )
    parameters = np.array([2.3, 1.7, 0.6, 8.0, 5.0])
    tau = np.column_stack(
        (
            parameters[0] * ddq[:, 0]
            + parameters[2] * (np.cos(q[:, 1]) * ddq[:, 1] - np.sin(q[:, 1]) * dq[:, 1] ** 2)
            + parameters[3] * np.sin(q[:, 0]),
            parameters[1] * ddq[:, 1]
            + parameters[2] * np.cos(q[:, 1]) * ddq[:, 0]
            + parameters[4] * np.sin(q[:, 1]),
        )
    )
    return q, dq, tau, parameters


class LowpassFilterTests(unittest.TestCase):
    def test_constant_input_has_zero_state_exponential_response(self):
        time = np.array([4.0, 4.03, 4.12, 4.31, 4.8, 5.2])
        values = np.broadcast_to(np.array([[2.0, -3.0], [0.5, 4.0]]), (len(time), 2, 2))
        rate = 3.7
        expected = (1.0 - np.exp(-rate * (time - time[0])))[:, None, None] * values
        assert_allclose(lowpass_filter(time, values, rate), expected, atol=2e-14)

    def test_ramp_input_matches_analytic_solution_on_irregular_times(self):
        time = np.array([7.0, 7.001, 7.025, 7.08, 7.3, 8.4])
        elapsed = time - time[0]
        rate, intercept, slope = 2.3, -1.2, 0.7
        values = intercept + slope * elapsed
        expected = intercept * (1.0 - np.exp(-rate * elapsed)) + slope * (
            elapsed - (1.0 - np.exp(-rate * elapsed)) / rate
        )
        assert_allclose(lowpass_filter(time, values, rate), expected, atol=2e-14)

    def test_invalid_rates_and_timestamps_are_rejected(self):
        for rate in (0.0, -1.0, np.nan, np.inf):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                lowpass_filter(np.array([0.0, 0.1]), np.ones(2), rate)
        for time in ([0.0, 0.0], [0.1, 0.0], [0.0, np.nan]):
            with self.subTest(time=time), self.assertRaises(ValueError):
                lowpass_filter(np.array(time), np.ones(2), 2.0)
        with self.assertRaises(ValueError):
            lowpass_filter(np.array([0.0, 0.1]), np.ones(3), 2.0)


class ResampleRecordingTests(unittest.TestCase):
    def test_duplicate_timestamps_keep_last_signal_values(self):
        time = np.array([2.0, 2.0, 2.1, 2.26])
        q = np.array([[99.0, 99.0], [0.0, 10.0], [1.0, 11.0], [2.6, 12.6]])
        tau = np.array([99.0, 0.0, 2.0, 5.2])
        uniform, q_uniform, tau_uniform = resample_recordings(time, q, tau, dt=0.1)
        assert_allclose(uniform, [0.0, 0.1, 0.2], atol=1e-14)
        assert_allclose(q_uniform, [[0.0, 10.0], [1.0, 11.0], [2.0, 12.0]], atol=1e-13)
        assert_allclose(tau_uniform, [0.0, 2.0, 4.0], atol=1e-13)
        self.assertLessEqual(uniform[-1], time[-1] - time[0])

    def test_single_unique_timestamp_retains_last_record(self):
        uniform, values = resample_recordings(
            np.array([3.0, 3.0]), np.array([[1.0], [2.0]]), dt=0.1
        )
        assert_allclose(uniform, [0.0])
        assert_allclose(values, [[2.0]])

    def test_decreasing_nonfinite_and_invalid_sample_period_are_rejected(self):
        for time in ([0.0, 0.1, 0.05], [0.0, np.nan, 0.2], []):
            with self.subTest(time=time), self.assertRaises(ValueError):
                resample_recordings(np.array(time), np.ones(len(time)), dt=0.1)
        for dt in (0.0, -0.1, np.nan, np.inf):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                resample_recordings(np.array([0.0, 0.1]), np.ones(2), dt=dt)
        with self.assertRaises(ValueError):
            resample_recordings(np.array([0.0, 0.1]), np.ones(3), dt=0.1)


class FilteredRegressorTests(unittest.TestCase):
    def test_variable_inertia_friction_and_nonzero_initial_velocity(self):
        time = np.linspace(0.0, 4.0, 4001)
        q, dq, tau, parameters = single_joint_recording(time)
        self.assertGreater(abs(dq[0, 0]), 0.5)
        h, filtered_tau = build_filtered_regressor(
            single_joint_regressor, time, q, dq, tau, len(parameters),
            friction_indices=(3, 4, 5), alpha=15.0, beta=10.0,
        )
        expected_tau = reference_filter(time, reference_filter(time, tau, 15.0), 10.0)
        assert_allclose(filtered_tau, expected_tau, atol=2e-12)
        assert_allclose(h @ parameters, expected_tau, atol=3e-5, rtol=2e-5)
        assert_allclose(h[0], 0.0, atol=1e-14)

    def test_coupled_kinetic_energy_requires_transposed_coriolis_term(self):
        time = np.linspace(0.0, 4.0, 4001)
        q, dq, tau, parameters = coupled_recording(time)
        h, filtered_tau = build_filtered_regressor(
            coupled_regressor, time, q, dq, tau, len(parameters), alpha=9.0, beta=7.0,
        )
        expected_tau = reference_filter(time, reference_filter(time, tau, 9.0), 7.0)
        assert_allclose(filtered_tau, expected_tau, atol=2e-12)
        assert_allclose(h @ parameters, expected_tau, atol=3e-5, rtol=2e-5)

    def test_beta_zero_skips_only_second_filter(self):
        time = np.linspace(0.0, 3.0, 3001)
        q, dq, tau, parameters = single_joint_recording(time)
        h, filtered_tau = build_filtered_regressor(
            single_joint_regressor, time, q, dq, tau, len(parameters),
            friction_indices=(3, 4, 5), alpha=8.0, beta=0.0,
        )
        expected_tau = reference_filter(time, tau, 8.0)
        assert_allclose(filtered_tau, expected_tau, atol=2e-12)
        assert_allclose(h @ parameters, expected_tau, atol=3e-5, rtol=2e-5)

    def test_parameters_recover_and_predict_a_held_out_trajectory(self):
        time = np.linspace(0.0, 12.0, 6001)
        q, dq, tau, parameters = single_joint_recording(time)
        h, filtered_tau = build_filtered_regressor(
            single_joint_regressor, time, q, dq, tau, len(parameters),
            friction_indices=(3, 4, 5), alpha=15.0, beta=10.0,
        )
        fit_h = h[500:].reshape(-1, len(parameters))
        fit_tau = filtered_tau[500:].reshape(-1)
        identified, _, rank, _ = np.linalg.lstsq(fit_h, fit_tau, rcond=None)
        self.assertEqual(rank, len(parameters))
        assert_allclose(identified, parameters, atol=3e-4, rtol=3e-4)

        held_time = np.linspace(0.0, 8.0, 4001)
        held_q, held_dq, held_tau, _ = single_joint_recording(held_time, phase=0.9)
        held_h, held_filtered_tau = build_filtered_regressor(
            single_joint_regressor, held_time, held_q, held_dq, held_tau, len(parameters),
            friction_indices=(3, 4, 5), alpha=15.0, beta=10.0,
        )
        residual = held_h[500:] @ identified - held_filtered_tau[500:]
        self.assertLess(np.sqrt(np.mean(residual**2)), 2e-4)

    def test_invalid_shapes_and_filter_settings_are_rejected(self):
        time = np.array([0.0, 0.1, 0.2])
        q, dq, tau, parameters = single_joint_recording(time)
        cases = (
            {"alpha": 0.0}, {"alpha": np.nan}, {"beta": -1.0}, {"beta": np.inf},
            {"friction_indices": (len(parameters),)},
        )
        for options in cases:
            with self.subTest(options=options), self.assertRaises(ValueError):
                build_filtered_regressor(
                    single_joint_regressor, time, q, dq, tau, len(parameters), **options
                )
        with self.assertRaises(ValueError):
            build_filtered_regressor(single_joint_regressor, time, q, dq[:2], tau, len(parameters))
        with self.assertRaises(ValueError):
            build_filtered_regressor(single_joint_regressor, time, q, dq, tau, 0)
        with self.assertRaises(ValueError):
            build_filtered_regressor(
                single_joint_regressor, np.array([0.0, 0.1, 0.1]), q, dq, tau, len(parameters)
            )
        with self.assertRaises(ValueError):
            build_filtered_regressor(
                single_joint_regressor, time[:1], q[:1], dq[:1], tau[:1], len(parameters)
            )


if __name__ == "__main__":
    unittest.main()
