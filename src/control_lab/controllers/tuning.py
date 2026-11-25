"""PID tuning rules for a first-order plus dead time (FOPDT) process.

Process model: G(s) = K exp(-L s) / (T s + 1), with K the process gain, T the time constant and
L the dead time (L is called theta in the models module). Every rule returns a dict
{"kp", "ti", "td"} for the textbook form Kp (1 + 1/(Ti s) + Td s); `ti = inf` means no integral
action and `td = 0` means no derivative.

The formulas are the commonly published forms (see research/algorithm-paper-index.md). Their
correctness here is checked by unit tests against hand computation, which tests the
transcription, not the engineering quality of the rule.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import brentq

INF = float("inf")


def _check(K, T, L):
    if K == 0 or T <= 0 or L <= 0:
        raise ValueError("K must be non-zero and T, L positive")


def ziegler_nichols_open(K, T, L, kind="PID"):
    """Ziegler-Nichols reaction-curve rules (1942)."""
    _check(K, T, L)
    base = T / (K * L)
    if kind == "P":
        return {"kp": base, "ti": INF, "td": 0.0}
    if kind == "PI":
        return {"kp": 0.9 * base, "ti": L / 0.3, "td": 0.0}
    if kind == "PID":
        return {"kp": 1.2 * base, "ti": 2.0 * L, "td": 0.5 * L}
    raise ValueError("kind must be P, PI or PID")


def ziegler_nichols_closed(Ku, Pu, kind="PID"):
    """Ziegler-Nichols ultimate-gain rules (1942)."""
    if Ku <= 0 or Pu <= 0:
        raise ValueError("Ku and Pu must be positive")
    if kind == "P":
        return {"kp": 0.5 * Ku, "ti": INF, "td": 0.0}
    if kind == "PI":
        return {"kp": 0.45 * Ku, "ti": Pu / 1.2, "td": 0.0}
    if kind == "PID":
        return {"kp": 0.6 * Ku, "ti": 0.5 * Pu, "td": 0.125 * Pu}
    raise ValueError("kind must be P, PI or PID")


def cohen_coon(K, T, L, kind="PID"):
    """Cohen-Coon rules (1953), quarter-amplitude damping for large L/T."""
    _check(K, T, L)
    r = L / T
    if kind == "P":
        return {"kp": (1.0 / K) * (T / L) * (1.0 + r / 3.0), "ti": INF, "td": 0.0}
    if kind == "PI":
        return {
            "kp": (1.0 / K) * (T / L) * (0.9 + r / 12.0),
            "ti": L * (30.0 + 3.0 * r) / (9.0 + 20.0 * r),
            "td": 0.0,
        }
    if kind == "PID":
        return {
            "kp": (1.0 / K) * (T / L) * (4.0 / 3.0 + r / 4.0),
            "ti": L * (32.0 + 6.0 * r) / (13.0 + 8.0 * r),
            "td": 4.0 * L / (11.0 + 2.0 * r),
        }
    raise ValueError("kind must be P, PI or PID")


def tyreus_luyben(Ku, Pu, kind="PID"):
    """Tyreus-Luyben ultimate-gain rules (1992): more conservative than Ziegler-Nichols."""
    if Ku <= 0 or Pu <= 0:
        raise ValueError("Ku and Pu must be positive")
    if kind == "PI":
        return {"kp": Ku / 3.2, "ti": 2.2 * Pu, "td": 0.0}
    if kind == "PID":
        return {"kp": Ku / 2.2, "ti": 2.2 * Pu, "td": Pu / 6.3}
    raise ValueError("kind must be PI or PID")


def imc_pid(K, T, L, lam, kind="PID"):
    """IMC-based PID (Rivera, Morari, Skogestad 1986) with a 1/1 Pade approximation of the delay.

    lam is the closed-loop filter time constant (s). A common starting point is lam > 0.2 L,
    larger values trade speed for robustness.
    """
    _check(K, T, L)
    if lam <= 0:
        raise ValueError("lam must be positive")
    if kind == "PI":
        return {"kp": T / (K * (lam + L)), "ti": T, "td": 0.0}
    if kind == "PID":
        return {
            "kp": (T + L / 2.0) / (K * (lam + L / 2.0)),
            "ti": T + L / 2.0,
            "td": T * L / (2.0 * T + L),
        }
    raise ValueError("kind must be PI or PID")


def lambda_tuning(K, T, L, lam):
    """Lambda (direct synthesis) PI tuning: Ti = T, Kp = T / (K (lam + L))."""
    _check(K, T, L)
    return {"kp": T / (K * (lam + L)), "ti": T, "td": 0.0}


def simc_pi(K, T, L, tau_c=None):
    """SIMC PI rule (Skogestad 2003): Kp = T / (K (tau_c + L)), Ti = min(T, 4 (tau_c + L))."""
    _check(K, T, L)
    tau_c = L if tau_c is None else tau_c
    return {"kp": T / (K * (tau_c + L)), "ti": min(T, 4.0 * (tau_c + L)), "td": 0.0}


def ultimate_point_fopdt(K, T, L):
    """Exact continuous-time ultimate gain Ku and period Pu of K exp(-L s)/(T s + 1).

    Solves the phase crossover  -w L - atan(w T) = -pi  and Ku = sqrt(1 + (w T)^2) / |K|.
    """
    _check(K, T, L)

    def phase(w):
        return w * L + math.atan(w * T) - math.pi

    w_u = brentq(phase, 1e-9, 1e4 / L)
    return math.sqrt(1.0 + (w_u * T) ** 2) / abs(K), 2.0 * math.pi / w_u


def relay_experiment(plant, d: float, Ts: float, t_end: float, hysteresis: float = 0.0,
                     setpoint: float = 0.0, bias: float = 0.0):
    """Relay feedback test (Astrom and Hagglund 1984) on a simulated plant.

    The relay output is bias +/- d. From the limit cycle of amplitude a and period Pu the
    describing-function estimate is Ku = 4 d / (pi sqrt(a^2 - h^2)) with hysteresis h.
    Returns (Ku, Pu, info). The estimate is approximate, not exact, for plants whose limit
    cycle is not sinusoidal.
    """
    plant.reset()
    n = int(round(t_end / Ts))
    y = np.empty(n)
    u_val = bias + d
    for k in range(n):
        e = setpoint - plant.y
        if e > hysteresis:
            u_val = bias + d
        elif e < -hysteresis:
            u_val = bias - d
        y[k] = plant.step(u_val)
    tail = y[n // 2 :]
    m = float(np.mean(tail))
    up = np.where((tail[:-1] < m) & (tail[1:] >= m))[0]
    if len(up) < 3:
        raise RuntimeError("no stable limit cycle found; increase t_end or check the relay amplitude")
    periods = np.diff(up) * Ts
    Pu = float(np.mean(periods[1:] if len(periods) > 2 else periods))
    a = 0.5 * float(np.mean([tail[up[i] : up[i + 1]].max() - tail[up[i] : up[i + 1]].min()
                              for i in range(len(up) - 1)]))
    Ku = 4.0 * d / (math.pi * math.sqrt(max(a * a - hysteresis**2, 1e-30)))
    return Ku, Pu, {"amplitude": a, "cycles": len(up) - 1, "period_std": float(np.std(periods))}
