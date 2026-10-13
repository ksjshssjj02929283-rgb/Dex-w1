import numpy as np


FRICTION_VELOCITY_SCALE = 0.1
STRIBECK_VELOCITY_SCALE = 0.1
STRIBECK_ALPHA = 2.0

FRICTION_MODEL_CLASSIC = "classic"
FRICTION_MODEL_SMOOTH = "smooth"
FRICTION_MODEL_STRIBECK = "stribeck"


def smooth_sign(velocity, velocity_scale=FRICTION_VELOCITY_SCALE):
    return np.tanh(np.asarray(velocity) / velocity_scale)


def friction_bases(velocity, model=FRICTION_MODEL_SMOOTH,
                   smooth_velocity=FRICTION_VELOCITY_SCALE,
                   stribeck_velocity=STRIBECK_VELOCITY_SCALE,
                   stribeck_alpha=STRIBECK_ALPHA):
    velocity = np.asarray(velocity, dtype=float)
    if model == FRICTION_MODEL_CLASSIC:
        return np.sign(velocity), np.ones_like(velocity)
    if model == FRICTION_MODEL_SMOOTH:
        return smooth_sign(velocity, smooth_velocity), np.ones_like(velocity)
    if model == FRICTION_MODEL_STRIBECK:
        return stribeck_basis(velocity, stribeck_velocity, stribeck_alpha, smooth_velocity)
    raise ValueError(f"未知摩擦模型: {model}")


def friction_settings(config):
    settings = {
        "friction_model": config.get("friction_model", FRICTION_MODEL_SMOOTH),
        "friction_velocity_scale": float(config.get("friction_velocity_scale", FRICTION_VELOCITY_SCALE)),
        "stribeck_velocity": float(config.get("stribeck_velocity", STRIBECK_VELOCITY_SCALE)),
        "stribeck_alpha": float(config.get("stribeck_alpha", STRIBECK_ALPHA)),
    }
    if settings["friction_model"] not in (FRICTION_MODEL_CLASSIC, FRICTION_MODEL_SMOOTH, FRICTION_MODEL_STRIBECK):
        raise ValueError(f"未知摩擦模型: {settings['friction_model']}")
    for key in ("friction_velocity_scale", "stribeck_velocity", "stribeck_alpha"):
        if not np.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f"{key} 必须是有限正数")
    return settings


def stribeck_basis(
    velocity,
    velocity_scale=STRIBECK_VELOCITY_SCALE,
    alpha=STRIBECK_ALPHA,
    smoothing_velocity=FRICTION_VELOCITY_SCALE,
):
    velocity = np.asarray(velocity)
    if not np.isfinite(velocity_scale) or velocity_scale <= 0:
        raise ValueError("Stribeck 特征速度必须是有限正数")
    if not np.isfinite(smoothing_velocity) or smoothing_velocity <= 0:
        raise ValueError("tanh 平滑速度必须是有限正数")
    static_part = np.exp(-np.power(np.abs(velocity) / velocity_scale, alpha))
    smooth_sign_value = smooth_sign(velocity, smoothing_velocity)
    return (1.0 - static_part) * smooth_sign_value, static_part * smooth_sign_value
