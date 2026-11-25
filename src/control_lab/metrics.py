"""Control performance metrics.

Definitions follow common textbook usage (Astrom and Murray, Feedback Systems; Ogata). Integral
metrics use trapezoidal integration on the supplied time grid. Functions return plain floats,
or None when a metric is undefined for the given signal (for example a response that never
reaches 90 percent of the step).
"""

from __future__ import annotations

import math

import numpy as np


def _trapz(y, x):
    # numpy 2 renamed trapz to trapezoid; support both
    fn = getattr(np, "trapezoid", None) or np.trapz
    return float(fn(y, x))


def iae(t, e) -> float:
    """Integral of absolute error."""
    return _trapz(np.abs(e), t)


def ise(t, e) -> float:
    """Integral of squared error."""
    return _trapz(np.square(e), t)


def itae(t, e) -> float:
    """Integral of time-weighted absolute error."""
    t = np.asarray(t, dtype=float)
    return _trapz(t * np.abs(e), t)


def itse(t, e) -> float:
    """Integral of time-weighted squared error."""
    t = np.asarray(t, dtype=float)
    return _trapz(t * np.square(e), t)


def mae(e) -> float:
    """Mean absolute error."""
    return float(np.mean(np.abs(e)))


def rmse(e) -> float:
    """Root mean square error."""
    return float(np.sqrt(np.mean(np.square(e))))


def control_energy(t, u) -> float:
    """Integral of u squared. Units: (input unit)^2 times second."""
    return _trapz(np.square(u), t)


def total_variation(u) -> float:
    """Sum of absolute input moves. A proxy for actuator wear and noise amplification."""
    return float(np.sum(np.abs(np.diff(u))))


def saturation_fraction(u, u_min: float, u_max: float, tol: float = 1e-9) -> float:
    """Share of samples at a limit (0..1)."""
    u = np.asarray(u)
    return float(np.mean((u <= u_min + tol) | (u >= u_max - tol)))


def ripple_peak_to_peak_percent(x) -> float:
    """(max - min) / |mean| * 100. Use on a steady-state window."""
    x = np.asarray(x, dtype=float)
    m = abs(float(np.mean(x)))
    return float("nan") if m == 0 else float((x.max() - x.min()) / m * 100.0)


def ripple_rms_percent(x) -> float:
    """Standard deviation / |mean| * 100. Use on a steady-state window."""
    x = np.asarray(x, dtype=float)
    m = abs(float(np.mean(x)))
    return float("nan") if m == 0 else float(np.std(x) / m * 100.0)


def _cross_time(t, yn, level, start=0):
    """First time yn crosses `level` upward after index `start` (linear interpolation)."""
    for k in range(max(start, 1), len(yn)):
        if yn[k - 1] < level <= yn[k]:
            frac = (level - yn[k - 1]) / (yn[k] - yn[k - 1])
            return float(t[k - 1] + frac * (t[k] - t[k - 1]))
    return None


def step_response_metrics(
    t, y, setpoint: float, y_initial: float | None = None, band: float = 0.02,
    steady_fraction: float = 0.05,
) -> dict:
    """Time-domain metrics of a step response.

    Quantities are normalised by the step size `setpoint - y_initial`.
      rise_time_10_90   time from 10 to 90 percent of the step (s)
      peak_time         time of the maximum (s)
      overshoot_percent max(0, peak - 1) * 100 of the step size
      undershoot_percent  depth below the initial value, percent of the step size
      settling_time     last time the response is outside +/- band of the final value (s);
                        None when it is still outside the band at the end
      steady_state_error  setpoint minus mean of the last `steady_fraction` of samples
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    y0 = float(y[0]) if y_initial is None else float(y_initial)
    step = float(setpoint) - y0
    if step == 0:
        raise ValueError("step size is zero")
    yn = (y - y0) / step
    n_tail = max(1, int(len(y) * steady_fraction))
    final = float(np.mean(yn[-n_tail:]))
    t10 = _cross_time(t, yn, 0.1)
    t90 = _cross_time(t, yn, 0.9)
    k_peak = int(np.argmax(yn))
    overshoot = max(0.0, float(yn[k_peak]) - 1.0) * 100.0
    undershoot = max(0.0, -float(np.min(yn))) * 100.0
    outside = np.where(np.abs(yn - 1.0) > band)[0]
    if len(outside) == 0:
        settling = 0.0
    elif outside[-1] == len(yn) - 1:
        settling = None
    else:
        settling = float(t[outside[-1] + 1])
    return {
        "rise_time_10_90": None if (t10 is None or t90 is None) else t90 - t10,
        "peak_time": float(t[k_peak]),
        "overshoot_percent": overshoot,
        "undershoot_percent": undershoot,
        "settling_time": settling,
        "steady_state_error": float(setpoint - (y0 + final * step)),
    }


def jsonable(obj):
    """Recursively convert numpy types and NaN to JSON-safe values (NaN becomes None)."""
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return jsonable(obj.tolist())
    if isinstance(obj, (np.floating, float)):
        f = float(obj)
        return None if (math.isnan(f) or math.isinf(f)) else f
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    return obj
