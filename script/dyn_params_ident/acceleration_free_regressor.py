"""Acceleration-free analytical filtering of robot dynamics regressors.

The regressor callable follows the usual torque convention:
``tau = H(q, dq, ddq) @ parameters``. Its parameter ordering is preserved.
"""

import operator

import numpy as np


def _positive_scalar(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a positive finite number") from exc
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return value


def _time_array(time, *, allow_repeated=False, min_samples=1):
    time = np.asarray(time, dtype=float)
    if time.ndim != 1 or time.size < min_samples:
        raise ValueError(f"time must be a one-dimensional array with at least {min_samples} samples")
    if not np.all(np.isfinite(time)):
        raise ValueError("time must contain only finite timestamps")
    with np.errstate(over="ignore", invalid="ignore"):
        steps = np.diff(time)
        elapsed = time - time[0]
    if not np.all(np.isfinite(steps)) or not np.all(np.isfinite(elapsed)):
        raise ValueError("the timestamp range must be finite")
    invalid = steps < 0.0 if allow_repeated else steps <= 0.0
    if np.any(invalid):
        order = "nondecreasing" if allow_repeated else "strictly increasing"
        raise ValueError(f"time must be {order}")
    return time


def _signal_array(values, samples, name):
    values = np.asarray(values, dtype=float)
    if values.ndim == 0 or values.shape[0] != samples:
        raise ValueError(f"{name} must have {samples} samples on axis 0")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must contain only finite values")
    return values


def resample_recordings(time, *signals, dt):
    """Resample recordings together onto a uniform grid without extrapolation.

    Repeated feedback timestamps retain the last sample in every signal.
    The returned time starts at zero, and each signal uses time on axis 0.
    """
    time = _time_array(time, allow_repeated=True)
    dt = _positive_scalar(dt, "dt")
    arrays = [
        _signal_array(signal, time.size, f"signal {index}")
        for index, signal in enumerate(signals)
    ]
    keep = np.r_[time[1:] != time[:-1], True]
    time = time[keep]
    arrays = [signal[keep] for signal in arrays]
    time = time - time[0]
    duration = time[-1]
    intervals = duration / dt
    if not np.isfinite(intervals):
        raise ValueError("the timestamp range is too large for dt")
    # Include a grid endpoint lost only to floating-point roundoff.
    count = int(np.floor(intervals + 4.0 * np.spacing(intervals))) + 1
    uniform_time = np.arange(count, dtype=float) * dt
    if uniform_time[-1] > duration:
        uniform_time[-1] = duration

    resampled = []
    for signal in arrays:
        columns = signal.reshape(time.size, -1)
        output = np.empty((uniform_time.size, columns.shape[1]), dtype=float)
        for index in range(columns.shape[1]):
            output[:, index] = np.interp(uniform_time, time, columns[:, index])
        resampled.append(output.reshape((uniform_time.size,) + signal.shape[1:]))
    return (uniform_time, *resampled)


def _lowpass_weights(step, rate):
    """Exact filter weights for a linearly interpolated input over one step."""
    with np.errstate(over="ignore"):
        z = rate * step
    decay = np.exp(-z)
    total = -np.expm1(-z)
    if z < 1e-4:
        current = z * (0.5 + z * (-1.0 / 6.0 + z * (1.0 / 24.0 + z * (-1.0 / 120.0 + z / 720.0))))
    else:
        current = 1.0 - total / z
    previous = total - current
    return decay, previous, current


def lowpass_filter(time, values, rate):
    """Apply zero-state causal ``rate / (s + rate)`` filtering on axis 0.

    Integration is exact when the input is piecewise linear between samples.
    Nonuniform, strictly increasing timestamps are supported. The first output
    is zero because the filter has no history before the first timestamp.
    """
    time = _time_array(time)
    values = _signal_array(values, time.size, "values")
    rate = _positive_scalar(rate, "rate")
    filtered = np.zeros_like(values, dtype=float)
    for index in range(1, time.size):
        decay, previous, current = _lowpass_weights(time[index] - time[index - 1], rate)
        filtered[index] = (
            decay * filtered[index - 1]
            + previous * values[index - 1]
            + current * values[index]
        )
    return filtered


def build_filtered_regressor(
    regressor_func,
    time,
    q,
    dq,
    tau,
    n_parameters,
    friction_indices=(),
    alpha=15.0,
    beta=10.0,
):
    """Build AF-ARF regression data using position, velocity, and torque only.

    Return ``(H_filtered, tau_filtered)`` with shapes ``(samples, dof,
    n_parameters)`` and ``(samples, dof)``. ``friction_indices`` identifies all
    friction columns in the original regressor; they are filtered separately
    from the inertial and gravity columns. Drive-inertia columns are retained.
    ``alpha`` and ``beta`` are filter rates in inverse seconds. ``beta=0``
    disables the optional second low-pass filter.
    """
    if not callable(regressor_func):
        raise ValueError("regressor_func must be callable")
    time = _time_array(time, min_samples=2)
    q = _signal_array(q, time.size, "q")
    dq = _signal_array(dq, time.size, "dq")
    tau = _signal_array(tau, time.size, "tau")
    if q.ndim != 2 or q.shape[1] == 0:
        raise ValueError("q must have shape (samples, dof) with at least one joint")
    if dq.shape != q.shape or tau.shape != q.shape:
        raise ValueError("q, dq, and tau must have the same (samples, dof) shape")
    try:
        n_parameters = operator.index(n_parameters)
    except TypeError as exc:
        raise ValueError("n_parameters must be a positive integer") from exc
    if n_parameters <= 0:
        raise ValueError("n_parameters must be a positive integer")
    alpha = _positive_scalar(alpha, "alpha")
    try:
        beta = float(beta)
    except (TypeError, ValueError) as exc:
        raise ValueError("beta must be a nonnegative finite number") from exc
    if not np.isfinite(beta) or beta < 0.0:
        raise ValueError("beta must be a nonnegative finite number")
    try:
        friction_indices = sorted({operator.index(index) for index in friction_indices})
    except (TypeError, ValueError) as exc:
        raise ValueError("friction_indices must contain integer parameter indices") from exc
    if any(index < 0 or index >= n_parameters for index in friction_indices):
        raise ValueError("friction_indices contains an out-of-range parameter index")

    dof = q.shape[1]
    zero = np.zeros(dof)
    identity = np.eye(dof)
    shape = (dof, n_parameters)

    def evaluate(position, velocity, auxiliary_acceleration):
        result = np.asarray(
            regressor_func(position, velocity, auxiliary_acceleration), dtype=float
        )
        if result.size != dof * n_parameters:
            raise ValueError("regressor_func returned an unexpected number of entries")
        result = result.reshape(shape).copy()
        if not np.all(np.isfinite(result)):
            raise ValueError("regressor_func returned nonfinite values")
        return result

    def without_friction(regressor):
        regressor[:, friction_indices] = 0.0
        return regressor

    filtered = np.empty((time.size,) + shape, dtype=float)
    integral_state = np.zeros(shape)
    friction_state = np.zeros((dof, len(friction_indices)))
    initial_momentum = None
    previous_input = None
    previous_friction = None

    # Polarize the velocity-quadratic term to recover columns of C, then
    # contract with dq to form C.T @ dq without differentiating velocity.
    for sample in range(time.size):
        position, velocity = q[sample], dq[sample]
        gravity = without_friction(evaluate(position, zero, zero))
        momentum = without_friction(evaluate(position, zero, velocity)) - gravity
        moving_regressor = evaluate(position, velocity, zero)
        friction = moving_regressor[:, friction_indices].copy()
        moving_regressor = without_friction(moving_regressor)
        coriolis_transpose = np.empty(shape)
        for joint in range(dof):
            unit = identity[:, joint]
            column = 0.5 * (
                without_friction(evaluate(position, velocity + unit, zero))
                - moving_regressor
                - without_friction(evaluate(position, unit, zero))
                + gravity
            )
            coriolis_transpose[joint] = velocity @ column
        integrand = alpha * momentum - gravity + coriolis_transpose
        if sample == 0:
            initial_momentum = momentum.copy()
        else:
            decay, previous, current = _lowpass_weights(
                time[sample] - time[sample - 1], alpha
            )
            integral_state = decay * integral_state + previous * previous_input + current * integrand
            friction_state = decay * friction_state + previous * previous_friction + current * friction
        elapsed = time[sample] - time[0]
        filtered[sample] = (
            alpha * momentum
            - integral_state
            - alpha * np.exp(-alpha * elapsed) * initial_momentum
        )
        filtered[sample, :, friction_indices] = friction_state.T
        previous_input = integrand
        previous_friction = friction

    filtered_torque = lowpass_filter(time, tau, alpha)
    if beta > 0.0:
        filtered = lowpass_filter(time, filtered, beta)
        filtered_torque = lowpass_filter(time, filtered_torque, beta)
    if not np.all(np.isfinite(filtered)) or not np.all(np.isfinite(filtered_torque)):
        raise ValueError("filter output is nonfinite; check the input scale and filter rates")
    return filtered, filtered_torque
