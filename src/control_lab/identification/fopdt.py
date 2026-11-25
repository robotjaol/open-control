"""Fit a first-order plus dead time model to a recorded step response.

Both fits assume the step is applied at t = 0 from rest and the response ends in steady state.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares


def fopdt_step(t, K: float, T: float, L: float, u_step: float = 1.0):
    """Step response of K exp(-L s)/(T s + 1) to an input step of size u_step."""
    t = np.asarray(t, dtype=float)
    return np.where(t > L, K * u_step * (1.0 - np.exp(-(t - L) / T)), 0.0)


def _first_crossing(t, y, level):
    idx = np.where(y >= level)[0]
    if len(idx) == 0 or idx[0] == 0:
        return None
    k = idx[0]
    frac = (level - y[k - 1]) / (y[k] - y[k - 1])
    return float(t[k - 1] + frac * (t[k] - t[k - 1]))


def fit_fopdt_two_point(t, y, u_step: float, steady_fraction: float = 0.1) -> dict:
    """Two-point method (Smith): times at 28.3 and 63.2 percent of the final value.

    T = 1.5 (t63 - t28),  L = t63 - T,  K = y_final / u_step.
    Sensitive to noise because it uses two samples; prefer the least-squares fit for noisy data.
    """
    t, y = np.asarray(t, dtype=float), np.asarray(y, dtype=float)
    n_tail = max(1, int(len(y) * steady_fraction))
    y_final = float(np.mean(y[-n_tail:]))
    t28 = _first_crossing(t, y, 0.283 * y_final)
    t63 = _first_crossing(t, y, 0.632 * y_final)
    if t28 is None or t63 is None:
        raise ValueError("response does not reach 63.2 percent of its final value")
    T = 1.5 * (t63 - t28)
    L = max(t63 - T, 0.0)
    K = y_final / u_step
    resid = y - fopdt_step(t, K, T, L, u_step)
    return {"K": K, "T": T, "L": L, "rmse": float(np.sqrt(np.mean(resid**2)))}


def fit_fopdt_least_squares(t, y, u_step: float, x0=None) -> dict:
    """Least-squares fit of (K, T, L) to the step response."""
    t, y = np.asarray(t, dtype=float), np.asarray(y, dtype=float)
    if x0 is None:
        try:
            g = fit_fopdt_two_point(t, y, u_step)
            x0 = [g["K"], max(g["T"], 1e-6), g["L"]]
        except ValueError:
            x0 = [float(y[-1]) / u_step, float(t[-1]) / 5.0, 0.0]

    def resid(p):
        return fopdt_step(t, p[0], p[1], p[2], u_step) - y

    sol = least_squares(resid, x0, bounds=([-np.inf, 1e-9, 0.0], [np.inf, np.inf, float(t[-1])]))
    K, T, L = (float(v) for v in sol.x)
    return {"K": K, "T": T, "L": L, "rmse": float(np.sqrt(np.mean(sol.fun**2)))}
