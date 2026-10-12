import sys
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dyn_params_ident.runtime_friction import (
    RuntimeFriction, read_runtime_friction, predict_runtime_friction,
)
from dyn_params_ident.acceleration_free_regressor import lowpass_filter
from dyn_params_ident.friction_model import friction_settings, friction_bases


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "robot_server_lib/dynamics/inverse_dynamics_symbol.cpp"
FV = [3, 13, 24, 35, 46, 57, 68]
FC = [index + 1 for index in FV]


def parameter_names():
    names = [f"inertia_{index}" for index in range(71)]
    for joint, index in enumerate(FV, 1):
        names[index:index + 3] = [f"fv_{joint}", f"fc_{joint}", f"fo_{joint}"]
    return names


class RuntimeFrictionTests(unittest.TestCase):
    def test_yaml_shared_stribeck_velocity_and_default(self):
        velocity = np.tile(np.array([[-0.1], [0.0], [0.1]]), (1, 7))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_config.yaml"
            for model in ("smooth", "stribeck"):
                config = {"friction_model": model, "friction_velocity_scale": 0.2,
                          "stribeck_velocity": 0.3, "stribeck_alpha": 1.0}
                path.write_text(yaml.safe_dump({"w1_v02": config}))
                rules = read_runtime_friction(SOURCE, "w1_v02", parameter_names(), path)
                self.assertEqual(rules.stribeck_velocity, 0.3)
                fc, fs = rules.bases(velocity)
                if model == "smooth":
                    np.testing.assert_allclose(fc, np.tanh(velocity / 0.2))
                    np.testing.assert_array_equal(fs, np.ones_like(velocity))
                else:
                    decay = np.exp(-np.abs(velocity) / 0.3)
                    np.testing.assert_allclose(fc, (1 - decay) * np.tanh(velocity / 0.3))
                    np.testing.assert_allclose(fs, decay * np.tanh(velocity / 0.3))
                path.write_text(yaml.safe_dump({"w1_v02": {"friction_model": "stribeck"}}))
                legacy = read_runtime_friction(SOURCE, "w1_v02", parameter_names(), path)
                self.assertEqual(legacy.stribeck_velocity, 0.1)
                decay = np.exp(-(np.abs(velocity) / 0.1) ** 2)
                np.testing.assert_allclose(legacy.bases(velocity)[0], (1 - decay) * np.tanh(velocity / 0.1))
                np.testing.assert_allclose(legacy.bases(velocity)[1], decay * np.tanh(velocity / 0.1))

    def test_obsolete_direction_velocity_does_not_change_stribeck(self):
        velocity = np.array([-0.1, 0.0, 0.1])
        for obsolete_velocity in (0.0, 0.05):
            settings = friction_settings({"friction_model": "stribeck", "stribeck_velocity": 0.3,
                                          "stribeck_smoothing_velocity": obsolete_velocity})
            self.assertNotIn("stribeck_smoothing_velocity", settings)
            fc, fs = friction_bases(velocity, settings["friction_model"],
                                    stribeck_velocity=settings["stribeck_velocity"])
            np.testing.assert_allclose(fc + fs, np.tanh(velocity / 0.3))

    def test_invalid_friction_velocities_are_rejected(self):
        for key in ("friction_velocity_scale", "stribeck_velocity", "stribeck_alpha"):
            for value in (0.0, -0.1, float("nan"), float("inf")):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    friction_settings({key: value})
        for value in (0.0, -0.1, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                friction_bases([0], "stribeck", stribeck_velocity=value)
        np.testing.assert_array_equal(friction_bases([0], "stribeck")[0], [0])

    def test_stribeck_replaces_both_friction_columns_before_filtering(self):
        velocity = np.array([[-0.1], [0.0], [0.1], [0.2]])
        rules = RuntimeFriction({}, (0,), 0.5, 0.0, (), 0.1, "stribeck", (1,))
        parameters = np.array([2.0, 4.0])
        matrix = np.ones((4, 2))
        decay = np.exp(-(np.abs(velocity) / 0.1) ** 2)
        expected = 0.5 * (2.0 + 2.0 * decay) * np.tanh(velocity / 0.1)
        np.testing.assert_allclose(predict_runtime_friction(matrix, parameters, velocity, rules), expected)
        context = {"t": np.arange(4) * 0.01, "dq": velocity, "alpha": 2.0, "beta": 0.0, "keep": slice(1, None)}
        np.testing.assert_allclose(
            predict_runtime_friction(matrix[1:], parameters, velocity[1:], rules, context),
            lowpass_filter(context['t'], expected, 2.0)[1:],
        )

    def test_tanh_basis_and_filtered_prediction(self):
        velocity = np.array([[-0.2], [-0.1], [0.0], [0.1], [0.2]])
        rules = RuntimeFriction({}, (0,), 0.2, 0.0, (), 0.1)
        expected = np.tanh(velocity / 0.1)
        np.testing.assert_allclose(rules.coulomb_basis(velocity), expected)
        parameters = np.array([3.0])
        regressor = expected.copy()
        prediction = predict_runtime_friction(regressor, parameters, velocity, rules)
        np.testing.assert_allclose(prediction, 0.6 * expected)
        time = np.arange(5) * 0.1
        context = {"t": time, "dq": velocity, "keep": slice(1, -1), "alpha": 2.0, "beta": 3.0}
        prediction = predict_runtime_friction(regressor[1:-1], parameters, velocity[1:-1], rules, context)
        filtered = lowpass_filter(time, lowpass_filter(time, expected, 2.0), 3.0)[1:-1]
        np.testing.assert_allclose(prediction, 0.6 * filtered)

    def test_both_generated_models_match_parameter_indices(self):
        for model in ("w1_v02", "w1_v03"):
            rules = read_runtime_friction(SOURCE, model, parameter_names())
            self.assertEqual(rules.viscous, dict(zip(FV, [0.6] * 5 + [0.5] * 2)))
            self.assertEqual(rules.coulomb_indices, tuple(FC))
            np.testing.assert_array_equal(rules.coulomb_scale, [0.2] * 7)
            self.assertEqual(rules.velocity_threshold, 0.0)
            self.assertEqual(rules.velocity_scale, 0.1)

    def test_wrong_qt_parameter_order_is_rejected(self):
        names = parameter_names()
        names[3], names[4] = names[4], names[3]
        with self.assertRaisesRegex(ValueError, "not a pure viscous"):
            read_runtime_friction(SOURCE, "w1_v03", names)

    def test_unsupported_scaling_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "inverse_dynamics_symbol.cpp"
            source.write_text(SOURCE.read_text().replace("fv_indices{3,", "unknown_indices{3,", 1))
            with self.assertRaises(ValueError):
                read_runtime_friction(source, "w1_v03", parameter_names())

    def test_yaml_arm_and_joint_ratios_and_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_config.yaml"
            config = {"w1_v02": {
                "fv_ratio_primary": [0, .1, .2, .3, .4, .5, .6],
                "fv_ratio_secondary": [.7, .8, .9, 1, 1.1, 1.2, 1.3],
                "fc_ratio_primary": [.1, .2, .3, .4, .5, .6, .7],
                "fc_ratio_secondary": [.8, .9, 1, 1.1, 1.2, 1.3, 1.4],
            }}
            path.write_text(yaml.safe_dump(config))
            for is_left, side in ((True, "primary"), (False, "secondary")):
                rules = read_runtime_friction(SOURCE, "w1_v02", parameter_names(), path, is_left)
                np.testing.assert_array_equal(list(rules.viscous.values()), config['w1_v02']['fv_ratio_' + side])
                np.testing.assert_array_equal(rules.coulomb_scale, config['w1_v02']['fc_ratio_' + side])
                dq = np.full((2, 7), 2.0)
                W = np.zeros((14, 71))
                for sample in range(2):
                    for j in range(7):
                        W[sample * 7 + j, FV[j]] = 2.0
                        W[sample * 7 + j, FC[j]] = 1.0
                        W[sample * 7 + j, FC[j] + 1] = 1.0
                parameters = np.ones(71)
                result = predict_runtime_friction(W, parameters, dq, rules)
                expected = 2 * np.array(list(rules.viscous.values())) + rules.coulomb_scale + 1.0
                np.testing.assert_allclose(result, np.tile(expected, (2, 1)))
            for bad in ([1, 2], [1, 1, -1, 1, 1, 1, 1], [float('nan')] * 7):
                config['w1_v02']['fc_ratio_primary'] = bad
                path.write_text(yaml.safe_dump(config))
                with self.assertRaises(ValueError):
                    read_runtime_friction(SOURCE, "w1_v02", parameter_names(), path)

    def test_standard_deadband_endpoints_and_no_parameter_mutation(self):
        dq = np.array([[-2.0], [-1.0], [0.0], [1.0], [2.0]])
        W = np.column_stack((np.ones(5), dq[:, 0], np.sign(dq[:, 0])))
        parameters = np.array([10.0, 2.0, 3.0])
        rules = RuntimeFriction({1: 0.5}, (2,), 0.2, 1.0, ())
        result = predict_runtime_friction(W, parameters, dq, rules)
        np.testing.assert_allclose(result[:, 0], [7.4, 9.0, 10.0, 11.0, 12.6])
        np.testing.assert_array_equal(parameters, [10.0, 2.0, 3.0])

    def test_af_deadband_is_filtered_before_crop(self):
        t = np.arange(20) * 0.1
        full_dq = np.ones((20, 1)) * 2.0
        full_dq[12:] = 0.5
        keep = slice(8, -2)
        filtered_sign = lowpass_filter(t, lowpass_filter(t, np.sign(full_dq), 2.0), 3.0)
        W = filtered_sign[keep]
        parameters = np.array([4.0])
        rules = RuntimeFriction({}, (0,), 0.2, 1.0, ())
        for beta in (0.0, 3.0):
            context = {"t": t, "dq": full_dq, "keep": keep, "alpha": 2.0, "beta": beta}
            result = predict_runtime_friction(W, parameters, full_dq[keep], rules, context)
            expected = lowpass_filter(t, np.where(full_dq > 1.0, 0.8, 0.0), 2.0)
            if beta:
                expected = lowpass_filter(t, expected, beta)
            np.testing.assert_allclose(result, expected[keep])
            self.assertGreater(result[0, 0], 0.0)


if __name__ == "__main__":
    unittest.main()
