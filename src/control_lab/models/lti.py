"""Linear plant helpers.

Plant protocol used by the simulation module:
    plant.y            current true output
    plant.step(u, d)   advance one controller sample with input u and disturbance d, return y
    plant.reset()      return to the initial state
"""

from __future__ import annotations

from collections import deque

import numpy as np
from scipy.linalg import expm


def c2d_zoh(A, B, Ts: float):
    """Exact zero-order-hold discretisation of x' = A x + B u.

    Returns (Ad, Bd) with Ad = exp(A Ts) and Bd = integral_0^Ts exp(A s) ds B, computed with the
    block-matrix exponential so that singular A (integrators) is handled.
    """
    A = np.atleast_2d(np.asarray(A, dtype=float))
    B = np.atleast_2d(np.asarray(B, dtype=float))
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    E = expm(M * Ts)
    return E[:n, :n], E[:n, n:]


def rk4_step(f, x, u, dt: float):
    """One classical Runge-Kutta step of x' = f(x, u) with u held constant over dt."""
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


class FOPDTPlant:
    """First-order plus dead time plant G(s) = K exp(-theta s) / (tau s + 1).

    Discretised exactly for a zero-order-hold input. The dead time is rounded to a whole number
    of samples, so `theta_effective = round(theta / Ts) * Ts`. A disturbance `d` is added to the
    input before the delay (an input disturbance).
    """

    def __init__(self, K: float, tau: float, theta: float, Ts: float, y0: float = 0.0):
        if tau <= 0 or Ts <= 0 or theta < 0:
            raise ValueError("tau and Ts must be positive and theta non-negative")
        self.K, self.tau, self.theta, self.Ts = float(K), float(tau), float(theta), float(Ts)
        self.delay_samples = int(round(theta / Ts))
        self.a = float(np.exp(-Ts / tau))
        self.y0 = float(y0)
        self.reset()

    @property
    def theta_effective(self) -> float:
        return self.delay_samples * self.Ts

    def reset(self) -> None:
        self.y = self.y0
        self._buffer = deque([0.0] * self.delay_samples, maxlen=max(self.delay_samples, 1))

    def step(self, u: float, d: float = 0.0) -> float:
        u_in = u + d
        if self.delay_samples > 0:
            u_delayed = self._buffer[0]
            self._buffer.append(u_in)
        else:
            u_delayed = u_in
        self.y = self.a * self.y + self.K * (1.0 - self.a) * u_delayed
        return self.y
