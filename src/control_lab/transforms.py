"""Reference-frame transforms and space vector PWM for three-phase machines.

Amplitude-invariant convention: a balanced set a = A cos(th), b = A cos(th - 120 deg),
c = A cos(th + 120 deg) maps to alpha = A cos(th), beta = A sin(th). In the rotating frame at
angle th the same set gives d = A, q = 0. Power is 1.5 times v_d i_d + v_q i_q in this convention.
"""

from __future__ import annotations

import numpy as np

SQRT3 = float(np.sqrt(3.0))


def clarke(a, b, c):
    """abc to alpha-beta. The zero-sequence (common-mode) part is rejected."""
    alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c)
    beta = (b - c) / SQRT3
    return alpha, beta


def inv_clarke(alpha, beta):
    """alpha-beta to abc with zero-sequence set to zero."""
    a = alpha
    b = -0.5 * alpha + 0.5 * SQRT3 * beta
    c = -0.5 * alpha - 0.5 * SQRT3 * beta
    return a, b, c


def park(alpha, beta, theta):
    """alpha-beta to dq in a frame rotated by `theta` (rad)."""
    ct, st = np.cos(theta), np.sin(theta)
    return alpha * ct + beta * st, -alpha * st + beta * ct


def inv_park(d, q, theta):
    """dq to alpha-beta."""
    ct, st = np.cos(theta), np.sin(theta)
    return d * ct - q * st, d * st + q * ct


def wrap_angle(theta):
    """Wrap an angle to [-pi, pi)."""
    return (theta + np.pi) % (2.0 * np.pi) - np.pi


def svpwm(v_alpha: float, v_beta: float, vdc: float):
    """Duty cycles for a two-level inverter using min-max zero-sequence injection.

    This is the standard carrier-based equivalent of space vector PWM (centred zero vectors).
    Returns (duty_abc, scale). `scale` < 1 means the request left the linear hexagon and the
    voltage vector was shortened, keeping its direction. The linear limit along the sector
    boundary is vdc / sqrt(3) in amplitude.
    """
    va, vb, vc = inv_clarke(v_alpha, v_beta)
    span = max(va, vb, vc) - min(va, vb, vc)
    scale = 1.0 if span <= vdc else vdc / span
    v = np.array([va, vb, vc]) * scale
    offset = -0.5 * (v.max() + v.min())
    duty = 0.5 + (v + offset) / vdc
    return np.clip(duty, 0.0, 1.0), scale


def duties_to_alpha_beta(duty, vdc: float):
    """Average phase voltages (relative to the DC midpoint) back to alpha-beta."""
    v = (np.asarray(duty) - 0.5) * vdc
    return clarke(v[0], v[1], v[2])
