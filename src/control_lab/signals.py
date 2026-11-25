"""Test signals, motion profiles and simple filters.

All functions are pure and deterministic. Random signals take an explicit seed.
"""

from __future__ import annotations

import numpy as np


def time_vector(t_end: float, dt: float) -> np.ndarray:
    """Return t = 0, dt, 2dt, ..., t_end (inclusive, rounded to the nearest sample)."""
    if dt <= 0 or t_end <= 0:
        raise ValueError("t_end and dt must be positive")
    n = int(round(t_end / dt)) + 1
    return np.arange(n) * dt


def step_signal(t: np.ndarray, amplitude: float = 1.0, t_step: float = 0.0) -> np.ndarray:
    """Step of height `amplitude` starting at `t_step`."""
    return np.where(np.asarray(t) >= t_step, float(amplitude), 0.0)


def white_noise(n: int, std: float, seed: int = 0) -> np.ndarray:
    """Zero-mean Gaussian noise with standard deviation `std` (same units as the signal)."""
    if std < 0:
        raise ValueError("std must be non-negative")
    return np.random.default_rng(seed).normal(0.0, std, size=n)


def quantize(x: np.ndarray | float, step: float):
    """Round to a uniform grid of width `step` (models an encoder or ADC resolution)."""
    if step <= 0:
        return x
    return np.round(np.asarray(x) / step) * step


def first_order_lowpass(x: np.ndarray, dt: float, tau: float) -> np.ndarray:
    """First-order low-pass y' = (x - y)/tau, discretised exactly for a sample-and-hold input."""
    if tau <= 0:
        return np.asarray(x, dtype=float).copy()
    a = np.exp(-dt / tau)
    y = np.empty(len(x), dtype=float)
    y[0] = x[0]
    for k in range(1, len(x)):
        y[k] = a * y[k - 1] + (1.0 - a) * x[k]
    return y


def moving_average(x: np.ndarray, window: int) -> np.ndarray:
    """Causal moving average. The first samples use the shorter available window."""
    if window < 1:
        raise ValueError("window must be >= 1")
    x = np.asarray(x, dtype=float)
    c = np.cumsum(np.insert(x, 0, 0.0))
    out = np.empty_like(x)
    for k in range(len(x)):
        lo = max(0, k + 1 - window)
        out[k] = (c[k + 1] - c[lo]) / (k + 1 - lo)
    return out


def ramp_speed_profile(t: np.ndarray, v_target: float, accel: float, t0: float = 0.0) -> np.ndarray:
    """Trapezoidal-velocity reference: constant acceleration up to `v_target` (infinite jerk)."""
    if accel <= 0:
        raise ValueError("accel must be positive")
    sign = 1.0 if v_target >= 0 else -1.0
    return sign * np.clip(accel * (np.asarray(t, dtype=float) - t0), 0.0, abs(v_target))


def scurve_speed_profile(
    t: np.ndarray, v_target: float, a_max: float, j_max: float, t0: float = 0.0
) -> np.ndarray:
    """Jerk-limited (S-curve) speed reference from 0 to `v_target` starting at `t0`.

    Acceleration is raised at `j_max` up to `a_max`, held, and lowered at `j_max` so the
    speed arrives at `v_target` with zero acceleration. Computed by forward integration
    with the braking rule "start reducing acceleration when v + a^2/(2 j) >= v_target".
    Assumes uniformly spaced `t`.
    """
    if a_max <= 0 or j_max <= 0:
        raise ValueError("a_max and j_max must be positive")
    t = np.asarray(t, dtype=float)
    dt = float(t[1] - t[0])
    v = np.zeros_like(t)
    acc = 0.0
    vel = 0.0
    sign = 1.0 if v_target >= 0 else -1.0
    target = abs(v_target)
    for k in range(len(t)):
        if t[k] >= t0:
            if vel + acc * acc / (2.0 * j_max) >= target:
                acc = max(acc - j_max * dt, 0.0)
            else:
                acc = min(acc + j_max * dt, a_max)
            vel = min(vel + acc * dt, target)
        v[k] = sign * vel
    return v
