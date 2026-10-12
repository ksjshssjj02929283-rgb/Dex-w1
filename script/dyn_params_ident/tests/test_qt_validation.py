import sys
import os
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import sympy
import sympybotics
import yaml
from PyQt5 import QtWidgets

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from qt_dyn_params_ident import DynParamCalculator, DynParamGUI, read_data, update_robot_config_section
from dyn_params_ident.runtime_friction import RuntimeFriction
from dyn_params_ident.acceleration_free_regressor import lowpass_filter, build_filtered_regressor
from dyn_params_ident.friction_model import friction_bases, friction_settings


class QtValidationTests(unittest.TestCase):
    def test_existing_tanh_control_shares_stribeck_velocity_and_restores_smooth_value(self):
        with patch.dict(os.environ, {"QT_QPA_PLATFORM": "offscreen"}):
            application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        characteristic_spin = QtWidgets.QDoubleSpinBox()
        tanh_spin = QtWidgets.QDoubleSpinBox()
        for spin in (characteristic_spin, tanh_spin):
            spin.setDecimals(6)
            spin.setRange(0.000001, 1000.0)
            spin.setValue(0.1)
        combo = QtWidgets.QComboBox()
        for model in ("smooth", "stribeck", "classic"):
            combo.addItem(model, model)
        left = DynParamCalculator("w1_v02", True, lambda message: None)
        right = DynParamCalculator("w1_v02", False, lambda message: None)
        for calculator in (left, right):
            calculator.set_friction_options({"friction_velocity_scale": 0.23, "stribeck_velocity": 0.11})
        gui = SimpleNamespace(
            friction_velocity_spins={"stribeck_velocity": characteristic_spin,
                                     "friction_velocity_scale": tanh_spin},
            friction_model_combo=combo,
            dyn=SimpleNamespace(calculator_left=left, calculator_right=right, traj_dir="/tmp"),
            traj_combo=SimpleNamespace(currentText=lambda: "轨迹1"),
            arm_combo=SimpleNamespace(currentText=lambda: "左臂"),
            stribeck_alpha_spin=SimpleNamespace(value=lambda: 2.0),
            identification_combo=SimpleNamespace(currentData=lambda: "standard", currentText=lambda: "传统回归"),
            af_alpha_spin=SimpleNamespace(value=lambda: 15.0), af_beta_spin=SimpleNamespace(value=lambda: 10.0),
            enable_buttons=lambda: None, log_msg=lambda message: None,
        )
        gui.update_params_from_ui = lambda: DynParamGUI.update_params_from_ui(gui)
        gui._load_friction_velocity_controls = lambda options: DynParamGUI._load_friction_velocity_controls(gui, options)
        gui.on_identification_options_changed = Mock(side_effect=lambda: DynParamGUI.on_identification_options_changed(gui))
        gui._load_friction_velocity_controls(left.friction_options)
        self.assertEqual(tanh_spin.value(), 0.23)
        self.assertFalse(characteristic_spin.isEnabled())
        combo.currentIndexChanged.connect(lambda index: DynParamGUI.on_friction_model_changed(gui))
        for key, spin in gui.friction_velocity_spins.items():
            spin.valueChanged.connect(lambda value, key=key: DynParamGUI.on_friction_velocity_changed(gui, key, value))
        combo.setCurrentIndex(combo.findData("stribeck"))
        self.assertTrue(tanh_spin.isEnabled())
        self.assertTrue(characteristic_spin.isEnabled())
        self.assertEqual(tanh_spin.value(), 0.11)
        gui.on_identification_options_changed.assert_called_once_with()
        for source, value in ((characteristic_spin, 0.19), (tanh_spin, 0.07)):
            with self.subTest(value=value):
                for calculator in (left, right):
                    calculator.solutions_beta_starextra = np.array([1.0])
                    calculator.proc_data["traj1"] = {}
                gui.on_identification_options_changed.reset_mock()
                source.setValue(value)
                self.assertEqual(characteristic_spin.value(), value)
                self.assertEqual(tanh_spin.value(), value)
                self.assertFalse(characteristic_spin.signalsBlocked())
                self.assertFalse(tanh_spin.signalsBlocked())
                gui.on_identification_options_changed.assert_called_once_with()
                for calculator in (left, right):
                    self.assertEqual(calculator.friction_options["stribeck_velocity"], value)
                    self.assertEqual(calculator.friction_options["friction_velocity_scale"], 0.23)
                    self.assertIsNone(calculator.solutions_beta_starextra)
                    self.assertEqual(calculator.proc_data, {})
                result = DynParamGUI._updated_identification_config(gui, "w1_v02:\n  friction_model: smooth\n", "w1_v02", {
                    "dyn_base_params_primary": [1.0], "dyn_base_params_secondary": [2.0],
                })
                section = yaml.safe_load(result)["w1_v02"]
                self.assertEqual(section["stribeck_velocity"], value)
                self.assertEqual(section["friction_velocity_scale"], 0.23)
                self.assertNotIn("stribeck_tanh_velocity", section)
                self.assertNotIn("stribeck_smoothing_velocity", section)
        combo.setCurrentIndex(combo.findData("smooth"))
        self.assertEqual(tanh_spin.value(), 0.23)
        self.assertTrue(tanh_spin.isEnabled())
        self.assertFalse(characteristic_spin.isEnabled())
        tanh_spin.setValue(0.31)
        for calculator in (left, right):
            self.assertEqual(calculator.friction_options["friction_velocity_scale"], 0.31)
            self.assertEqual(calculator.friction_options["stribeck_velocity"], 0.07)
        combo.setCurrentIndex(combo.findData("classic"))
        self.assertFalse(tanh_spin.isEnabled())
        self.assertFalse(characteristic_spin.isEnabled())
        combo.setCurrentIndex(combo.findData("stribeck"))
        self.assertEqual(tanh_spin.value(), 0.07)
        self.assertEqual(characteristic_spin.value(), 0.07)
        combo.setCurrentIndex(combo.findData("smooth"))
        self.assertEqual(tanh_spin.value(), 0.31)

    def test_full_validation_equals_mass_coriolis_gravity_and_friction(self):
        definition = sympybotics.RobotDef("test", [(0, 1, 0, "q"), (0, 1, 0, "q")], dh_convention="standard")
        definition.frictionmodel = {"Coulomb", "viscous", "offset"}
        definition.driveinertiamodel = "simplified"
        definition.gravityacc = sympy.Matrix([0, -9.81, 0])
        robot = sympybotics.RobotDynCode(definition)
        robot.calc_base_parms()
        parameters = np.arange(robot.dyn.n_base, dtype=float) * 0.05 + 0.2
        full_parameters = np.asarray(robot.dyn.Pb, dtype=float) @ parameters
        evaluators = {}
        for name, code in (("mass", robot.M_code), ("gravity", robot.g_code), ("coriolis", robot.c_code)):
            namespace = {"math": math, "sin": np.sin, "cos": np.cos}
            exec(sympybotics.robot_code_to_func("python", code, "result", name, definition), namespace)
            evaluators[name] = namespace[name]
        q = np.array([[0.2, 0.3], [-0.3, 0.6], [0.4, -0.3]])
        dq = np.array([[0.2, -0.1], [-0.3, 0.4], [0.1, 0.2]])
        ddq = np.array([[0.3, -0.4], [0.4, 0.2], [-0.2, 0.5]])
        inertial = np.array([
            np.asarray(evaluators["mass"](full_parameters, position)).reshape(2, 2) @ acceleration
            for position, acceleration in zip(q, ddq)
        ])
        rigid_zero = np.array([
            np.asarray(evaluators["gravity"](full_parameters, position)) +
            np.asarray(evaluators["coriolis"](full_parameters, position, velocity))
            for position, velocity in zip(q, dq)
        ])
        self.assertGreater(np.max(np.abs(inertial)), 0.01)
        names = list(robot.dyn.dynparms)
        fv, fc, fo = [full_parameters[[names.index(symbol) for symbol in symbols]]
                      for symbols in (definition.fv, definition.fc, definition.fo)]
        base_names = list(map(str, robot.dyn.baseparms))
        for model, shared_velocity in (("classic", 0.19), ("smooth", 0.19), ("stribeck", 0.19), ("stribeck", 0.07)):
            options = {"friction_model": model, "friction_velocity_scale": 0.23,
                       "stribeck_velocity": shared_velocity, "stribeck_alpha": 1.5}
            if model == "stribeck":
                decay = np.exp(-(np.abs(dq) / shared_velocity) ** 1.5)
                fc_basis = (1 - decay) * np.tanh(dq / shared_velocity)
                fo_basis = decay * np.tanh(dq / shared_velocity)
            else:
                fc_basis, fo_basis = friction_bases(dq, model, 0.23)
            zero_expected = rigid_zero + dq * fv + fc_basis * fc + fo_basis * fo
            full_expected = inertial + zero_expected
            rules = RuntimeFriction(
                {base_names.index(str(symbol)): 1.0 for symbol in definition.fv},
                tuple(base_names.index(str(symbol)) for symbol in definition.fc), 1.0, 0.0, (), 0.23,
                model, tuple(base_names.index(str(symbol)) for symbol in definition.fo),
                shared_velocity, 1.5,
            )
            for algorithm in ("standard", "af_arf"):
                calculator = DynParamCalculator("w1_v02", True, lambda message: None)
                calculator.ndof = 2
                calculator.rbt = robot
                calculator.identification_algorithm = algorithm
                calculator.set_friction_options(options)
                calculator.solutions_beta_starextra = parameters.reshape(-1, 1)
                regressor = calculator.create_regressor_func(robot, calculator.friction_options)
                full_regression = np.vstack([
                    np.asarray(regressor(position, velocity, acceleration)).reshape(2, -1)
                    for position, velocity, acceleration in zip(q, dq, ddq)
                ])
                calculator.proc_data["traj2"] = {
                    "t": np.arange(3) * 0.01, "q": q, "dq": dq, "ddq": ddq,
                    "W": full_regression[:, robot.dyn.base_idxs],
                    "tau": full_expected, "tau_current": full_expected,
                    "tau_sensor": np.zeros_like(full_expected), "friction_context": None,
                    "algorithm": algorithm,
                }
                with patch.object(calculator, "get_validation_friction_rules", return_value=rules):
                    predictions = calculator.get_all_validation_predictions()
                for key in ("full_unscaled", "full_scaled"):
                    np.testing.assert_allclose(predictions[key], full_expected, atol=1e-10)
                for key in ("zero_unscaled", "zero_scaled"):
                    np.testing.assert_allclose(predictions[key], zero_expected, atol=1e-10)

    def test_three_friction_models_and_invalidation(self):
        definition = sympybotics.RobotDef("test", [(0, 0, 0, "q")], dh_convention="standard")
        definition.frictionmodel = {"Coulomb", "viscous", "offset"}
        robot = sympybotics.RobotDynCode(definition)
        fc_index = list(robot.dyn.dynparms).index(definition.fc[0])
        fo_index = list(robot.dyn.dynparms).index(definition.fo[0])
        for model in ("classic", "smooth", "stribeck"):
            regressor = DynParamCalculator.create_regressor_func(robot, {"friction_model": model})
            for velocity in (-0.1, 0.0, 0.1):
                actual = regressor([0.0], [velocity], [0.0])
                if model == "stribeck":
                    decay = np.exp(-(abs(velocity) / 0.1) ** 2)
                    expected = ((1 - decay) * np.tanh(velocity / 0.1), decay * np.tanh(velocity / 0.1))
                else:
                    expected = (np.sign(velocity) if model == "classic" else np.tanh(velocity / 0.1), 1.0)
                np.testing.assert_allclose([actual[fc_index], actual[fo_index]], expected)
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        calculator.solutions_beta_starextra = np.array([1.0])
        calculator.proc_data["traj1"] = {}
        self.assertTrue(calculator.set_friction_options({"friction_model": "stribeck"}))
        self.assertIsNone(calculator.solutions_beta_starextra)
        self.assertEqual(calculator.proc_data, {})

    def test_yaml_update_is_scoped_and_preserves_comments(self):
        content = "# header\nw1_v02:\n  friction_model: smooth # keep\n  dyn_base_params_primary: [1,\n    2]\nw1_v03:\n  dyn_base_params_primary: [3]\n"
        result = update_robot_config_section(content, "w1_v02", {
            "friction_model": "stribeck", "stribeck_velocity": 0.1,
            "dyn_base_params_primary": [4.0, 5.0],
        })
        config = yaml.safe_load(result)
        self.assertEqual(config['w1_v02']['dyn_base_params_primary'], [4, 5])
        self.assertEqual(config['w1_v03']['dyn_base_params_primary'], [3])
        self.assertIn("# keep", result)
        with self.assertRaises(ValueError):
            update_robot_config_section(content, "missing", {"friction_model": "smooth"})

    def test_repository_config_update_preserves_other_version(self):
        path = Path(__file__).resolve().parents[3] / "w1_server/robot_config.yaml"
        content = path.read_text()
        original = yaml.safe_load(content)
        updated = update_robot_config_section(content, "w1_v02", {
            "friction_model": "stribeck", "dyn_base_params_primary": [0.0] * 71,
        })
        config = yaml.safe_load(updated)
        self.assertEqual(config["w1_v03"], original["w1_v03"])
        self.assertEqual(config["w1_v02"]["friction_model"], "stribeck")
        self.assertEqual(config["w1_v02"]["dyn_base_params_secondary"], original["w1_v02"]["dyn_base_params_secondary"])

    def test_single_arm_write_rejects_changed_shared_friction_model(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        calculator.set_friction_options({"friction_model": "stribeck"})
        gui = SimpleNamespace(dyn=SimpleNamespace(calculator_left=calculator))
        content = "w1_v02:\n  friction_model: smooth\n"
        with self.assertRaises(ValueError):
            DynParamGUI._updated_identification_config(gui, content, "w1_v02", {
                "dyn_base_params_primary": [1.0],
            })
        result = DynParamGUI._updated_identification_config(gui, content, "w1_v02", {
            "dyn_base_params_primary": [1.0], "dyn_base_params_secondary": [2.0],
        })
        self.assertEqual(yaml.safe_load(result)["w1_v02"]["friction_model"], "stribeck")

    def test_editing_ui_velocities_invalidates_both_arms_and_writes_yaml(self):
        left = DynParamCalculator("w1_v02", True, lambda message: None)
        right = DynParamCalculator("w1_v02", False, lambda message: None)
        for calculator in (left, right):
            calculator.set_friction_options({"friction_velocity_scale": 0.23})
            calculator.solutions_beta_starextra = np.array([1.0])
            calculator.proc_data["traj1"] = {}
        values = {"friction_velocity_scale": 0.23, "stribeck_velocity": 0.19}
        gui = SimpleNamespace(
            dyn=SimpleNamespace(calculator_left=left, calculator_right=right, traj_dir="/tmp"),
            traj_combo=SimpleNamespace(currentText=lambda: "轨迹1"),
            arm_combo=SimpleNamespace(currentText=lambda: "左臂"),
            friction_model_combo=SimpleNamespace(currentData=lambda: "stribeck", currentText=lambda: "Stribeck"),
            stribeck_alpha_spin=SimpleNamespace(value=lambda: 1.5),
            friction_velocity_spins={key: SimpleNamespace(value=lambda: 0.19) for key in values},
            identification_combo=SimpleNamespace(currentData=lambda: "standard", currentText=lambda: "传统回归"),
            af_alpha_spin=SimpleNamespace(value=lambda: 15.0), af_beta_spin=SimpleNamespace(value=lambda: 10.0),
            enable_buttons=lambda: None, log_msg=lambda message: None,
        )
        gui.update_params_from_ui = lambda: DynParamGUI.update_params_from_ui(gui)
        DynParamGUI.on_identification_options_changed(gui)
        for calculator in (left, right):
            self.assertIsNone(calculator.solutions_beta_starextra)
            self.assertEqual(calculator.proc_data, {})
            for key, value in values.items():
                self.assertEqual(calculator.friction_options[key], value)
        self.assertFalse(gui._left_calculated)
        self.assertFalse(gui._right_calculated)
        content = ("w1_v02:\n  friction_model: smooth\n  stribeck_smoothing_velocity: 0.07 # obsolete\n"
                   "w1_v03:\n  stribeck_smoothing_velocity: 0.05 # other version\n")
        parameters = {"dyn_base_params_primary": [1.0], "dyn_base_params_secondary": [2.0]}
        result = DynParamGUI._updated_identification_config(gui, content, "w1_v02", parameters)
        section = yaml.safe_load(result)["w1_v02"]
        self.assertEqual(friction_settings(section), left.friction_options)
        self.assertNotIn("stribeck_smoothing_velocity", section)
        self.assertEqual(yaml.safe_load(result)["w1_v03"]["stribeck_smoothing_velocity"], 0.05)
        self.assertIn("# other version", result)
        for key, value in parameters.items():
            self.assertEqual(section[key], value)
        with self.assertRaises(ValueError):
            DynParamGUI._updated_identification_config(gui, content, "w1_v02", {"dyn_base_params_primary": [1.0]})

    def test_generated_regressor_uses_tanh_instead_of_copysign(self):
        definition = sympybotics.RobotDef("test", [(0, 0, 0, "q")], dh_convention="standard")
        definition.frictionmodel = {"Coulomb", "viscous", "offset"}
        robot = sympybotics.RobotDynCode(definition)
        regressor = DynParamCalculator.create_regressor_func(robot)
        fc_index = list(robot.dyn.dynparms).index(definition.fc[0])
        for velocity in (-0.2, -0.1, 0.0, 0.1, 0.2):
            self.assertAlmostEqual(regressor([0.0], [velocity], [0.0])[fc_index],
                                   np.tanh(velocity / 0.1))

    def test_zero_acceleration_uses_only_gravity_coriolis_and_friction(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        fv, fc, fo = sympy.symbols("fv_1 fc_1 fo_1")
        calculator.rbt = SimpleNamespace(
            dyn=SimpleNamespace(Pb=np.eye(3), dynparms=[fv, fc, fo], baseparms=[fv, fc, fo]),
            rbtdef=SimpleNamespace(fv=[fv], fc=[fc], fo=[fo]),
        )
        calculator.solutions_beta_starextra = np.array([2.0, 3.0, 0.4])
        calculator._feedforward_functions = (
            lambda parameters, q: np.array([10.0]),
            lambda parameters, q, dq: dq ** 2,
        )
        dq = np.array([[2.0], [0.5], [-2.0]])
        calculator.proc_data["traj2"] = {
            "q": np.zeros_like(dq), "dq": dq, "t": np.arange(3),
            "tau_current": np.zeros_like(dq), "tau_sensor": np.zeros_like(dq),
            "tau": np.full_like(dq, 7.0),
            # There is deliberately no W, ddq, or inertia evaluator.
        }
        rules = RuntimeFriction({0: 0.5}, (1,), 0.2, 1.0, ())
        with patch("qt_dyn_params_ident.read_runtime_friction", return_value=rules):
            _, _, _, baseline, runtime, target, _ = calculator.get_zero_acceleration_validation()
            np.testing.assert_array_equal(target, np.full_like(dq, 7.0))
            for scales in ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (1.5, 0.7, 0.3)):
                custom = calculator.get_validation_friction_rules(scales)
                custom_result = calculator.get_zero_acceleration_validation(custom)
                np.testing.assert_array_equal(custom_result[3], baseline)
                expected = (10.0 + dq ** 2 + 2.0 * dq * scales[0]
                            + 3.0 * np.where(np.abs(dq) > 1.0, np.sign(dq), 0) * scales[2] + 0.4)
                np.testing.assert_allclose(custom_result[4], expected)
                calculator.proc_data["traj2"].update({
                    "W": np.column_stack((dq[:, 0], np.sign(dq[:, 0]), np.ones(3))),
                    "tau": np.zeros_like(dq), "algorithm": "standard", "friction_context": None,
                })
                calculator.solutions_beta_starextra = np.array([[2.0], [3.0], [0.4]])
                full = calculator.get_friction_ratio_validation(custom)
                np.testing.assert_allclose(full[4], expected - (10.0 + dq ** 2))
                calculator.solutions_beta_starextra = calculator.solutions_beta_starextra.reshape(-1)
        np.testing.assert_allclose(baseline[:, 0], [21.4, 14.65 - 3.0 * (1.0 - np.tanh(5.0)), 7.4])
        np.testing.assert_allclose(runtime[:, 0], [17.0, 11.15, 11.8])
        np.testing.assert_array_equal(calculator.solutions_beta_starextra, [2.0, 3.0, 0.4])

    def test_custom_scales_follow_joint_names_and_preserve_source_rules(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        calculator.rbt = SimpleNamespace(dyn=SimpleNamespace(baseparms=["fv_7", "fv_2", "fv_6"]))
        rules = RuntimeFriction({0: 0.5, 1: 0.6, 2: 0.5}, (3,), 0.2, 0.001, ())
        with patch("qt_dyn_params_ident.read_runtime_friction", return_value=rules):
            custom = calculator.get_validation_friction_rules((0.8, 0.4, 0.1))
            self.assertEqual(custom.viscous, {0: 0.4, 1: 0.8, 2: 0.4})
            self.assertEqual(custom.velocity_threshold, rules.velocity_threshold)
            self.assertEqual(custom.coulomb_scale, 0.1)
            self.assertIs(calculator.get_validation_friction_rules(), rules)
            self.assertEqual(rules.viscous, {0: 0.5, 1: 0.6, 2: 0.5})
            for invalid in ((-1, 1, 1), (1, float('nan'), 1), (1, 1)):
                with self.assertRaises(ValueError):
                    calculator.get_validation_friction_rules(invalid)

    def test_zero_only_cache_preserves_raw_torque_for_both_algorithms(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        calculator.ndof = 1
        time = np.arange(700) * calculator.dt
        position = np.sin(time).reshape(-1, 1)
        raw_torque = (np.sin(time) + 0.5 * np.sin(2 * np.pi * 20 * time)).reshape(-1, 1)
        sensor = raw_torque * 2
        recording = (time, position, raw_torque, sensor, time, position)
        keep = slice(300, -100)
        for algorithm in ("standard", "af_arf"):
            calculator.identification_algorithm = algorithm
            with patch("qt_dyn_params_ident.read_data", return_value=recording), patch.object(Path, "is_file", return_value=True):
                calculator._cache_zero_acceleration_validation_data()
            proc = calculator.proc_data["traj2"]
            np.testing.assert_array_equal(proc["tau"], raw_torque[keep])
            np.testing.assert_allclose(proc["tau_current"], raw_torque[keep])
            self.assertIsNone(proc["friction_context"])
            self.assertEqual(proc["evaluation_domain"], "instantaneous")
            if algorithm == "standard":
                standard_proc = proc
            else:
                for key in ("t", "q", "dq", "tau", "tau_sensor"):
                    np.testing.assert_array_equal(proc[key], standard_proc[key])
            self.assertNotIn("W", proc)

    def test_standard_training_and_validation_use_filtered_states_and_raw_torque(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        calculator.ndof = 1
        fv, fc, fo = sympy.symbols("fv_1 fc_1 fo_1")
        calculator.rbt = SimpleNamespace(
            dof=1,
            dyn=SimpleNamespace(n_dynparms=3, base_idxs=[0, 1, 2], Pb=np.eye(3),
                                dynparms=[fv, fc, fo], baseparms=[fv, fc, fo]),
            rbtdef=SimpleNamespace(fv=[fv], fc=[fc], fo=[fo]),
        )
        calculator.solutions_beta_starextra = np.array([[2.0], [3.0], [0.4]])
        calculator._feedforward_functions = (
            lambda parameters, position: np.array([10.0]),
            lambda parameters, position, velocity: velocity ** 2,
        )
        time = np.arange(700) * calculator.dt
        position = (np.sin(time) + 0.1 * np.sin(2 * np.pi * 20 * time)).reshape(-1, 1)
        raw_torque = (np.cos(time) + 0.5 * np.sin(2 * np.pi * 20 * time)).reshape(-1, 1)
        recording = (time, position, raw_torque, np.zeros_like(raw_torque), time, position)
        cutoff = calculator.get_filter_cutoff()
        expected_q, expected_dq, expected_ddq, filtered_torque = calculator.diff_and_filt_data(
            1, calculator.dt, position, raw_torque, cutoff, cutoff, cutoff, cutoff,
        )
        keep = slice(300, -100)
        regressor = lambda q, dq, ddq: [q[0], dq[0], ddq[0]]
        with patch("qt_dyn_params_ident.read_data", return_value=recording), patch.object(calculator, "create_regressor_func", return_value=regressor):
            for traj in ("traj1", "traj2"):
                calculator._cache_trajectory(traj)
                proc = calculator.proc_data[traj]
                np.testing.assert_allclose(proc["q"], expected_q[keep])
                np.testing.assert_allclose(proc["dq"], expected_dq[keep])
                np.testing.assert_allclose(proc["ddq"], expected_ddq[keep])
                np.testing.assert_array_equal(proc["tau"], raw_torque[keep])
                np.testing.assert_array_equal(proc["omega"], raw_torque[keep])
                np.testing.assert_allclose(proc["W"], np.column_stack((expected_q[keep], expected_dq[keep], expected_ddq[keep])))
        self.assertGreater(np.max(np.abs(filtered_torque[keep] - raw_torque[keep])), 0.1)
        rules = RuntimeFriction({0: 0.5}, (1,), 0.2, 0.0, (), 0.1)
        with patch.object(calculator, "get_validation_friction_rules", return_value=rules):
            predictions = calculator.get_all_validation_predictions()
        np.testing.assert_array_equal(predictions["measured_full"], raw_torque[keep])
        np.testing.assert_array_equal(predictions["measured_zero"], raw_torque[keep])
        np.testing.assert_array_equal(predictions["t_full"], predictions["t_zero"])

        standard_proc = calculator.proc_data["traj2"]
        calculator.identification_algorithm = "af_arf"
        for alpha, beta in ((15.0, 10.0), (40.0, 0.0)):
            calculator.af_alpha, calculator.af_beta = alpha, beta
            with patch("qt_dyn_params_ident.read_data", return_value=recording), patch.object(calculator, "create_regressor_func", return_value=regressor), patch.object(calculator, "process_acceleration_free_data", side_effect=AssertionError("Validation must not use AF filters")):
                calculator.cache_validation_data()
            for key in ("t", "q", "dq", "ddq", "W", "tau", "omega"):
                np.testing.assert_array_equal(calculator.proc_data["traj2"][key], standard_proc[key])
            self.assertIsNone(calculator.proc_data["traj2"]["friction_context"])
            self.assertEqual(calculator.proc_data["traj2"]["evaluation_domain"], "instantaneous")
            with patch.object(calculator, "get_validation_friction_rules", return_value=rules):
                af_predictions = calculator.get_all_validation_predictions()
            for key in ("measured_full", "measured_zero", "full_unscaled", "full_scaled", "zero_unscaled", "zero_scaled"):
                np.testing.assert_array_equal(af_predictions[key], predictions[key])

    def test_af_zero_predictions_and_full_predictions_share_raw_target(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        calculator.ndof = 1
        fv, fc, fo = sympy.symbols("fv_1 fc_1 fo_1")
        calculator.rbt = SimpleNamespace(
            dyn=SimpleNamespace(Pb=np.eye(3), dynparms=[fv, fc, fo], baseparms=[fv, fc, fo]),
            rbtdef=SimpleNamespace(fv=[fv], fc=[fc], fo=[fo]),
        )
        calculator.solutions_beta_starextra = np.array([[2.0], [3.0], [0.4]])
        calculator._feedforward_functions = (
            lambda parameters, position: np.array([10.0]),
            lambda parameters, position, velocity: velocity ** 2,
        )
        time = np.arange(30) * 0.01
        velocity = np.linspace(-0.2, 0.2, 30).reshape(-1, 1)
        keep = slice(8, -2)
        raw_target = np.full((30, 1), 20.0)
        rules = RuntimeFriction({0: 0.5}, (1,), 0.2, 0.0, (), 0.1)
        for beta in (0.0, 10.0):
            calculator.identification_algorithm = "af_arf"
            calculator.af_beta = beta
            target = raw_target[keep]
            calculator.proc_data["traj2"] = {
                "t": time[keep], "q": np.zeros_like(velocity[keep]), "dq": velocity[keep],
                "tau_current": raw_target[keep], "tau_sensor": np.zeros_like(target),
                "tau": target, "tau_sensor_filtered": None,
                "algorithm": "af_arf", "W": np.zeros((len(target), 3)),
                "friction_context": None,
            }
            with patch.object(calculator, "get_validation_friction_rules", return_value=rules):
                predictions = calculator.get_all_validation_predictions()
            np.testing.assert_array_equal(predictions["measured_full"], target)
            np.testing.assert_array_equal(predictions["measured_zero"], target)
            np.testing.assert_array_equal(predictions["t_full"], predictions["t_zero"])
            baseline = 10.0 + velocity ** 2 + 2.0 * velocity + 3.0 * np.tanh(velocity / 0.1) + 0.4
            runtime = 10.0 + velocity ** 2 + velocity + 0.6 * np.tanh(velocity / 0.1) + 0.4
            np.testing.assert_allclose(predictions["zero_unscaled"], baseline[keep])
            np.testing.assert_allclose(predictions["zero_scaled"], runtime[keep])

    def test_af_training_still_uses_filtered_equation_without_acceleration(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        calculator.ndof = 1
        calculator.identification_algorithm = "af_arf"
        fv, fc, fo = sympy.symbols("fv_1 fc_1 fo_1")
        calculator.rbt = SimpleNamespace(
            dof=1, dyn=SimpleNamespace(n_dynparms=3, base_idxs=[0, 1, 2], dynparms=[fv, fc, fo]),
            rbtdef=SimpleNamespace(fv=[fv], fc=[fc], fo=[fo]),
        )
        time = np.arange(700) * calculator.dt
        position = (np.sin(time) + 0.1 * np.sin(2 * np.pi * 20 * time)).reshape(-1, 1)
        raw_torque = (np.cos(time) + 0.5 * np.sin(2 * np.pi * 20 * time)).reshape(-1, 1)
        timestamps = time + 0.002 * np.sin(np.arange(700))
        timestamps[350] = timestamps[349]
        recording = (timestamps, position, raw_torque, raw_torque * 2, time, position)
        regressor = lambda q, dq, ddq: [dq[0], np.tanh(dq[0] / 0.1), 1.0]
        calculator.identification_algorithm = "standard"
        with patch("qt_dyn_params_ident.read_data", return_value=recording), patch.object(calculator, "create_regressor_func", return_value=regressor):
            calculator._cache_trajectory("traj1")
        standard_proc = calculator.proc_data["traj1"]
        calculator.identification_algorithm = "af_arf"
        with patch("qt_dyn_params_ident.read_data", return_value=recording), patch.object(calculator, "create_regressor_func", return_value=regressor), patch("qt_dyn_params_ident.build_filtered_regressor", wraps=build_filtered_regressor) as af_builder, patch.object(calculator, "central_diff", wraps=calculator.central_diff) as differentiate:
            calculator._cache_trajectory("traj1")
        af_builder.assert_called_once()
        self.assertEqual(differentiate.call_count, 1)
        proc = calculator.proc_data["traj1"]
        self.assertIsNone(proc["ddq"])
        for key in ("t", "t_raw", "q_raw", "q", "dq", "tau_current", "tau_sensor"):
            np.testing.assert_allclose(proc[key], standard_proc[key], atol=1e-12)
        np.testing.assert_array_equal(af_builder.call_args.args[1], time)
        np.testing.assert_array_equal(af_builder.call_args.args[2][300:-100], standard_proc["q"])
        np.testing.assert_array_equal(af_builder.call_args.args[3][300:-100], standard_proc["dq"])
        np.testing.assert_array_equal(af_builder.call_args.args[4], raw_torque)
        np.testing.assert_array_equal(differentiate.call_args.args[0], position[:, 0])
        self.assertEqual(differentiate.call_args.args[1], 0.01)
        expected = lowpass_filter(time, raw_torque, calculator.af_alpha)
        expected = lowpass_filter(time, expected, calculator.af_beta)
        np.testing.assert_allclose(proc["tau"], expected[300:-100])
        np.testing.assert_allclose(proc["tau_current"], raw_torque[300:-100])
        self.assertGreater(np.max(np.abs(proc["tau"] - proc["tau_current"])), 0.1)

    def test_full_validation_rebuilds_zero_only_cache(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        calculator.rbt = SimpleNamespace()
        calculator.solutions_beta_starextra = np.array([[2.0]])
        calculator.proc_data["traj2"] = {"zero_acceleration_only": True}

        def cache_full_data():
            calculator.proc_data["traj2"] = {
                "W": np.ones((3, 1)), "t": np.arange(3), "tau": np.full((3, 1), 5.0),
                "tau_current": np.full((3, 1), 6.0), "tau_sensor": np.zeros((3, 1)),
                "algorithm": "standard",
            }

        with patch.object(Path, "is_file", return_value=True), patch.object(calculator, "cache_validation_data", side_effect=cache_full_data) as cache:
            result = calculator.get_regr_validatation()
        cache.assert_called_once()
        np.testing.assert_array_equal(result[3], np.full((3, 1), 2.0))
        np.testing.assert_array_equal(result[4], np.full((3, 1), 5.0))

    def test_independent_files_and_invalidation(self):
        calculator = DynParamCalculator("w1_v02", True, lambda message: None)
        identified = np.array([1.0])
        calculator.solutions_beta_starextra = identified
        calculator.proc_data = {"traj1": {"training": True}, "traj2": {"old": True}}
        calculator.set_recording_file("traj2", "/tmp/other_dir/validation.dat")
        self.assertIs(calculator.solutions_beta_starextra, identified)
        self.assertIn("traj1", calculator.proc_data)
        self.assertNotIn("traj2", calculator.proc_data)
        calculator.set_recording_file("traj1", "/tmp/training_dir/record.dat")
        self.assertIsNone(calculator.solutions_beta_starextra)
        self.assertEqual(calculator.proc_data, {})
        self.assertEqual(calculator.recording_path("traj1"), Path("/tmp/training_dir/record.dat"))
        self.assertEqual(calculator.recording_path("traj2"), Path("/tmp/other_dir/validation.dat"))

    def test_recording_does_not_require_reference_file(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "arbitrary_name.txt"
            recording = np.arange(60, dtype=float).reshape(4, 15)
            np.savetxt(filename, recording)
            t, q, current, sensor, _, reference = read_data(7, 0.01, filename, None)
            np.testing.assert_array_equal(q, recording[:, 1:8])
            np.testing.assert_array_equal(current, recording[:, 8:15])
            np.testing.assert_array_equal(sensor, np.zeros_like(current))
            np.testing.assert_array_equal(reference, q)


if __name__ == "__main__":
    unittest.main()
