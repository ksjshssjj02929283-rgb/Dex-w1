"""Read the supported symbolic-runtime friction rules for offline validation."""

from dataclasses import dataclass
from pathlib import Path
import re

import numpy as np
import yaml

from .acceleration_free_regressor import lowpass_filter
from .friction_model import smooth_sign


@dataclass(frozen=True)
class RuntimeFriction:
    viscous: dict
    coulomb_indices: tuple
    coulomb_scale: object  # Scalar for legacy callers, or one ratio per joint.
    velocity_threshold: float
    sources: tuple
    velocity_scale: float = None

    def coulomb_basis(self, velocity):
        if self.velocity_scale is not None:
            return smooth_sign(velocity, self.velocity_scale)
        return np.where(np.abs(velocity) > self.velocity_threshold, np.sign(velocity), 0.0)


def _function_body(text, name):
    match = re.search(r"\b" + name + r"\s*\([^)]*\)\s*\{", text)
    if not match:
        raise ValueError(f"Missing runtime function: {name}")
    start = match.end()
    depth = 1
    for index in range(start, len(text)):
        depth += (text[index] == "{") - (text[index] == "}")
        if depth == 0:
            return text[start:index]
    raise ValueError(f"Unclosed runtime function: {name}")


def _without_comments(text):
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)


def read_runtime_friction(source, robot_type, base_parameters, config_path=None, is_left=True):
    source = Path(source)
    if robot_type not in ("w1_v02", "w1_v03"):
        raise ValueError(f"Unsupported runtime model: {robot_type}")
    generated = source.parent / "inverse_dynamics_symbol_generated" / robot_type / "regression_func.cpp"
    body = _function_body(_without_comments(source.read_text(encoding="utf-8")), "InvDynamikBySymbol")
    names = list(map(str, base_parameters))
    def indices(key):
        match = re.search(r"\b" + key + r"\s*\{([\d,\s]+)\}", body)
        if not match:
            raise ValueError(f"Missing runtime parameter mapping: {key}")
        return [int(value) for value in match[1].split(',')]
    fv_indices, runtime_fc_indices = indices("fv_indices"), indices("fc_indices")
    if len(fv_indices) != 7 or len(runtime_fc_indices) != 7:
        raise ValueError("Expected seven friction indices")
    if config_path is None:
        config_path = source.parents[2] / "w1_server/robot_config.yaml"
    config_path = Path(config_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))[robot_type]
    side = "primary" if is_left else "secondary"
    def ratios(kind, default):
        values = np.asarray(config.get(f"{kind}_ratio_{side}", default), dtype=float)
        if values.shape != (7,) or not np.all(np.isfinite(values)) or np.any(values < 0):
            raise ValueError(f"{kind}_ratio_{side} requires seven finite nonnegative ratios")
        return values
    fv_ratios = ratios("fv", [0.6] * 5 + [0.5] * 2)
    fc_ratios = ratios("fc", [0.2] * 7)
    viscous = {}
    for joint, (index, factor) in enumerate(zip(fv_indices, fv_ratios), 1):
        if index >= len(names) or names[index] != f"fv_{joint}":
            raise ValueError(f"Runtime index {index} is not a pure viscous parameter in the Qt model")
        viscous[index] = factor
    if not viscous:
        raise ValueError("No runtime viscous friction assignments found")

    code = _without_comments(generated.read_text(encoding="utf-8"))
    sign_body = _function_body(code, "sign")
    sign_match = re.fullmatch(
        r"\s*double\s+f_c_scale\s*=\s*([^;]+);\s*"
        r"double\s+vel_threshold\s*=\s*([^;]+);\s*"
        r"if\s*\(x\s*>\s*vel_threshold\)\s*return\s+f_c_scale;\s*"
        r"else\s+if\s*\(x\s*<\s*-vel_threshold\)\s*return\s+-f_c_scale;\s*"
        r"else\s*return\s*0\.0;\s*", sign_body,
    )
    tanh_match = re.fullmatch(
        r"\s*const\s+double\s+velocity_scale\s*=\s*([^;]+);\s*"
        r"return\s+tanh\(x\s*/\s*velocity_scale\);\s*", sign_body,
    )
    velocity_scale = None
    if tanh_match:
        velocity_scale = float(tanh_match.group(1))
        if not np.isfinite(velocity_scale) or velocity_scale <= 0:
            raise ValueError("Invalid runtime tanh velocity scale")
        scale, threshold = 1.0, 0.0
    elif sign_match:
        scale, threshold = map(float, sign_match.groups())
    else:
        raise ValueError("Unsupported runtime Coulomb sign rule; check generated source")
    if not np.all(np.isfinite([scale, threshold])) or scale < 0 or threshold < 0:
        raise ValueError("Invalid runtime Coulomb scale or velocity threshold")
    if scale != 1.0:
        raise ValueError("Generated Coulomb sign must be unscaled; ratios come from YAML")
    dof = sum(bool(re.fullmatch(r"fc_\d+", name)) for name in names)
    fc_indices = []
    for joint in range(dof):
        fv_index = names.index(f"fv_{joint + 1}")
        fc_index = names.index(f"fc_{joint + 1}")
        if runtime_fc_indices[joint] != fc_index:
            raise ValueError("Runtime and Qt Coulomb parameter indices differ")
        # Both generated arm functions must use the same parameter mapping.
        fv_term = rf"dq\[{joint}\]\*dyn_base_params\[{fv_index}\]"
        fc_term = rf"dyn_base_params\[{fc_index}\]\*\(sign\(dq\[{joint}\]\)\)"
        if len(re.findall(fv_term, code)) != 2 or len(re.findall(fc_term, code)) != 2:
            raise ValueError(f"Generated and Qt friction ordering differ at joint {joint + 1}")
        if fv_index not in viscous:
            raise ValueError(f"Missing viscous scaling for joint {joint + 1}")
        fc_indices.append(fc_index)
    if len(viscous) != dof or dof == 0:
        raise ValueError("Incomplete runtime friction mapping")
    return RuntimeFriction(viscous, tuple(fc_indices), fc_ratios, threshold,
                           (config_path, source, generated), velocity_scale)


def predict_runtime_friction(W, parameters, dq, rules, filter_context=None):
    """Change only friction; apply its velocity basis before AF filters and cropping."""
    W = np.asarray(W, dtype=float)
    parameters = np.asarray(parameters, dtype=float).reshape(-1)
    dq = np.asarray(dq, dtype=float)
    velocity = dq if filter_context is None else filter_context["dq"]
    basis = rules.coulomb_scale * rules.coulomb_basis(velocity)
    if filter_context is not None:
        basis = lowpass_filter(filter_context["t"], basis, filter_context["alpha"])
        if filter_context["beta"] > 0:
            basis = lowpass_filter(filter_context["t"], basis, filter_context["beta"])
        basis = basis[filter_context["keep"]]
    if basis.shape != dq.shape or dq.shape[1] != len(rules.coulomb_indices):
        raise ValueError("Friction basis and validation trajectory have different shapes")
    scaled = parameters.copy()
    for index, factor in rules.viscous.items():
        scaled[index] *= factor
    scaled[list(rules.coulomb_indices)] = 0.0
    prediction = (W @ scaled).reshape(dq.shape)
    prediction += basis * parameters[list(rules.coulomb_indices)]
    return prediction
