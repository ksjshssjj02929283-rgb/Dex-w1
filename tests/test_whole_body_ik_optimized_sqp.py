"""Offline regression tests for the W1 standard SQP formulation.

Optional kinematics dependencies and the W1V02X URDF are required. Set
W1_IK_TEST_URDF to use a URDF outside the usual local robot-data cache.
"""
import ast
import logging
import os
from pathlib import Path
import sys
import types

import numpy as np
import pytest

ca = pytest.importorskip('casadi')
pin = pytest.importorskip('pinocchio')
pytest.importorskip('pinocchio.casadi')

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src/dexe_robot_client/w1/kinematics/whole_body_ik.py'
REF = np.array([.8316, -1.6631, .8389, .0501, -.0316, -.4996,
                -1.5202, -1.1717, 1.5875, -.5347, -.0131, .087,
                -.0489, 1.515, .9311, -1.585, .4383, .0153, .1507, .0465])
LEFT = np.array([[.0009, -.115, .9934, .5548], [-1., -.0006, .0008, .0433],
                 [.0005, -.9934, -.115, 1.092], [0., 0., 0., 1.]])
RIGHT = np.array([[.0009, .115, .9934, .5545], [1., -.0006, -.0008, -.3852],
                  [.0005, .9934, -.115, 1.0918], [0., 0., 0., 1.]])
HOME = np.array([.785, -1.57, .9, 0., 0., -.4, -1.57, -.785, 1.57,
                 -.785, 0., 0., 0., 1.57, .785, -1.57, .785, 0., 0., 0.])


@pytest.fixture(scope='module')
def ik_module():
    # Avoid package startup, robot connections, UI imports and the file logger.
    tree = ast.parse(SOURCE.read_text())
    tree.body = [node for node in tree.body if not (
        isinstance(node, ast.ImportFrom) and node.module == 'common' and node.level == 3)]
    module = types.ModuleType('_optimized_sqp_test_module')
    module.get_logger = lambda: logging.getLogger('optimized_sqp_test')
    sys.modules[module.__name__] = module
    exec(compile(tree, str(SOURCE), 'exec'), module.__dict__)
    yield module
    sys.modules.pop(module.__name__, None)


@pytest.fixture(scope='module')
def urdf():
    path = Path(os.environ.get('W1_IK_TEST_URDF',
                str(Path.home() / '.cache/dexe_data/assembled/w1/robot.urdf')))
    if not path.is_file():
        pytest.skip('W1V02X URDF unavailable; set W1_IK_TEST_URDF')
    return str(path)


def build(m, urdf, *, optimized=True, locked=None, posture=None, options=None):
    if options is None:
        options = m.SqpMethodOptions(use_gauss_newton=optimized,
                                    eliminate_leg_equality=optimized)
    return m.WholeBodyIK(
        urdf, m.TrackingFrameConfig.init_from_urdf('W1V02X', urdf),
        m.JointNameConfig.init_from_robot('W1V02X'), solver_options=options,
        freezed_joint_dict=locked, posture_config=posture)


def set_parameters(ik, reference=REF, left=LEFT, right=RIGHT):
    op = ik.opti
    op.set_value(ik.var_q_last, reference)
    op.set_value(ik.param_left_arm_tf, left)
    op.set_value(ik.param_right_arm_tf, right)
    op.set_value(ik.param_upper_body_leaning_threshold, 0.)
    op.set_value(ik.param_eye_focus_point, [.6, 0., .8])
    op.set_value(ik.param_eye_focus_weight, 1.)
    op.set_value(ik.param_translation_error_threshold, 1e6)
    op.set_value(ik.param_rotation_error_threshold, 1e5)


@pytest.mark.parametrize('shape', [(20,), (1, 20), (20, 1)])
def test_reported_failure_converges_and_returns_full_q(ik_module, urdf, shape):
    ik = build(ik_module, urdf)
    assert (ik.opti.nx, ik.opti.ng) == (19, 28)
    assert ik.gauss_newton_residual.shape == (52, 1)
    # Structural zeros in the mapping must stay sparse; a dense zero-filled
    # matrix makes the QP Hessian dense and can trigger QRQP working-set cycles.
    assert ca.jacobian(ik.var_q, ik.var_q_independent).nnz() == 20

    ok, q = ik.solve_ik(LEFT, RIGHT, ref_qpos=REF.reshape(shape),
                        eye_focus_point=np.array([.6, 0., .8]))

    assert ok
    assert q.shape == (20,)
    assert np.all(np.isfinite(q))
    assert q[1] == pytest.approx(-2 * q[0], abs=1e-12)
    np.testing.assert_array_equal(ik.opti.value(ik.var_q_last), REF)
    np.testing.assert_array_equal(ik.cache_solution_q, q)
    assert ik.opti.stats()['iter_count'] < 20
    assert ik.opti.value(ik.opti.f) == pytest.approx(1.826501138215, abs=1e-8)
    g = ik.opti.value(ik.opti.g)
    lo, hi = ik.opti.value(ik.opti.lbg), ik.opti.value(ik.opti.ubg)
    assert np.max(np.maximum(np.maximum(lo-g, g-hi), 0.)) <= 1e-6
    grad = ca.gradient(ik.opti.f, ik.opti.x)
    jac = ca.jacobian(ik.opti.g, ik.opti.x)
    assert np.max(np.abs(ik.opti.value(grad + jac.T @ ik.opti.lam_g))) <= 1e-6


@pytest.mark.parametrize('forward', [True, False])
def test_elimination_preserves_cost_and_linear_bounds(ik_module, urdf, forward):
    m = ik_module
    posture = m.PostureConfig(leg_leaning_forward=forward)
    full = build(m, urdf, optimized=False, posture=posture)
    reduced = build(m, urdf, posture=posture)
    for ik in (full, reduced):
        set_parameters(ik)
    full_fn = ca.Function('full_problem', [full.opti.x, full.opti.p],
                          [full.opti.f, full.opti.g])
    red_fn = ca.Function('reduced_problem', [reduced.opti.x, reduced.opti.p],
                         [reduced.opti.f, reduced.opti.g, reduced.var_q,
                          reduced.gauss_newton_residual])
    p_full, p_red = full.opti.value(full.opti.p), reduced.opti.value(reduced.opti.p)
    lo_full, hi_full = full.opti.value(full.opti.lbg), full.opti.value(full.opti.ubg)
    lo_red, hi_red = reduced.opti.value(reduced.opti.lbg), reduced.opti.value(reduced.opti.ubg)
    rng = np.random.default_rng(17)
    for ankle in np.linspace(-1.7, 1.7, 101):
        z = rng.normal(size=19)
        z[0] = ankle
        f, g, q, residual = red_fn(z, p_red)
        expected_f, expected_g = full_fn(q, p_full)
        assert float(f) == pytest.approx(float(expected_f), rel=1e-12, abs=1e-10)
        assert float(ca.sumsqr(residual)) == pytest.approx(float(f), rel=1e-12, abs=1e-10)
        # Check the transformed ankle/knee bounds directly, without unrelated
        # constraints masking errors in the bound intersection.
        rows = [0, 1, 22, 23]
        original = np.asarray(expected_g).ravel()[rows]
        old_feasible = np.all((original >= lo_full[rows]) & (original <= hi_full[rows]))
        new_feasible = lo_red[0] <= float(g[0]) <= hi_red[0]
        assert old_feasible == new_feasible


@pytest.mark.parametrize('locked', [
    {'ANKLE': .785}, {'KNEE': -1.57},
    {'ANKLE': .785, 'KNEE': -1.57, 'BUTTOCK': .9},
])
def test_locked_leg_pair_keeps_existing_constraints(ik_module, urdf, locked):
    full = build(ik_module, urdf, optimized=False)
    ik = build(ik_module, urdf, locked=locked)
    assert not ik._leg_equality_eliminated
    assert ik.opti.nx == ik.robot_.model.nq
    reference = np.zeros(ik.robot_.model.nq)
    for name in ik.robot_.model.names[1:]:
        reference[ik.joint_index_resolver.q_index(name)] = HOME[full.joint_index_resolver.q_index(name)]
    pin.framesForwardKinematics(ik.robot_.model, ik.robot_.data, reference)
    left = ik.robot_.data.oMf[ik.left_arm_tracking_id].homogeneous.copy()
    right = ik.robot_.data.oMf[ik.right_arm_tracking_id].homogeneous.copy()

    ok, q = ik.solve_ik(left, right, ref_qpos=reference,
                        eye_focus_point=np.array([.6, 0., .8]))

    assert ok
    assert q.shape == reference.shape
    np.testing.assert_array_equal(ik.opti.value(ik.var_q_last), reference)


def test_custom_problem_does_not_inherit_standard_leg_equality(ik_module, urdf):
    m = ik_module
    ik = build(m, urdf)
    ik.register_problem('custom', lambda ctx: m.IKProblemSpec('custom', costs=[
        m.IKCostTerm.from_residual('custom_posture', lambda c: c.q - c.q_last)]))
    ik.set_problem('custom')
    assert ik.opti.nx == 20
    assert not ik._leg_equality_eliminated
    ok, q = ik.solve_ik(LEFT, RIGHT, ref_qpos=REF)
    assert ok
    np.testing.assert_allclose(q, REF, atol=1e-12)
    ik.set_problem('standard')
    assert (ik.opti.nx, ik.opti.ng) == (19, 28)


def test_gn_requires_explicit_residual_for_custom_cost(ik_module, urdf):
    m = ik_module
    ik = build(m, urdf)
    ik.register_problem('legacy', lambda ctx: m.IKProblemSpec('legacy', costs=[
        m.IKCostTerm('posture', lambda c: ca.sumsqr(c.q - c.q_last))]))
    with pytest.raises(ValueError, match='needs a residual'):
        ik.set_problem('legacy')
    ik.solver_options.use_gauss_newton = False
    ik.set_problem('legacy')
    ok, q = ik.solve_ik(LEFT, RIGHT, ref_qpos=REF)
    assert ok
    np.testing.assert_allclose(q, REF, atol=1e-12)


def test_ipopt_build_retains_original_coordinates(ik_module, urdf):
    ik = build(ik_module, urdf, options=ik_module.IpoptOptions())
    assert (ik.opti.nx, ik.opti.ng) == (20, 32)
    assert not ik._leg_equality_eliminated
    assert ik.gauss_newton_residual is None


def test_invalid_gn_damping_is_rejected(ik_module, urdf):
    with pytest.raises(ValueError, match='gn_damping'):
        build(ik_module, urdf, options=ik_module.SqpMethodOptions(
            use_gauss_newton=True, gn_damping=-1.))


def test_joint_mapping_uses_model_indices_not_fixed_positions(ik_module):
    m = ik_module
    ik = m.WholeBodyIK.__new__(m.WholeBodyIK)
    ik.robot_ = types.SimpleNamespace(model=types.SimpleNamespace(nq=4))
    ik.joint_name_config = types.SimpleNamespace(ankle_joint='ankle', knee_joint='knee')
    ik.joint_index_resolver = types.SimpleNamespace(q_index={'ankle': 2, 'knee': 0}.get)
    ik.solver_options = m.SqpMethodOptions(eliminate_leg_equality=True)
    ik.opti = ca.Opti()
    ik._create_joint_variables('standard')
    mapping = ca.Function('joint_mapping', [ik.var_q_independent], [ik.var_q])

    q = np.asarray(mapping([2., 3., 4.])).ravel()

    np.testing.assert_array_equal(q, [-6., 2., 3., 4.])
    assert ca.jacobian(ik.var_q, ik.var_q_independent).nnz() == 4
