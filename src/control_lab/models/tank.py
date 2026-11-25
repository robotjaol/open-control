"""Single tank with gravity outflow, a standard process-control benchmark.

    A dh/dt = q_in - c sqrt(h) - q_d

h level (m), q_in pump inflow (m^3/s), q_d extra outflow demand (m^3/s, the disturbance),
A cross-section (m^2), c outlet coefficient (m^2.5/s). Parameters are illustrative, not taken
from a specific plant.
"""

from __future__ import annotations

import numpy as np

from control_lab.models.lti import c2d_zoh, rk4_step


class TankLevel:
    def __init__(self, area=0.8, c_out=0.004, h0=1.0, q_max=0.008, Ts=5.0, substeps=10):
        if area <= 0 or c_out <= 0 or Ts <= 0:
            raise ValueError("area, c_out and Ts must be positive")
        self.area, self.c_out, self.q_max = float(area), float(c_out), float(q_max)
        self.Ts, self.substeps, self.h0 = float(Ts), int(substeps), float(h0)
        self.reset()

    def reset(self) -> None:
        self.h = self.h0

    @property
    def y(self) -> float:
        return self.h

    def _f(self, x, u):
        q_in, q_d = u
        h = max(x[0], 0.0)
        return np.array([(q_in - self.c_out * np.sqrt(h) - q_d) / self.area])

    def step(self, q_in: float, q_d: float = 0.0) -> float:
        q_in = float(np.clip(q_in, 0.0, self.q_max))
        x = np.array([self.h])
        dt = self.Ts / self.substeps
        for _ in range(self.substeps):
            x = rk4_step(self._f, x, (q_in, q_d), dt)
            x[0] = max(x[0], 0.0)
        self.h = float(x[0])
        return self.h

    def equilibrium_inflow(self, h: float) -> float:
        """Inflow that holds level `h` with no disturbance."""
        return self.c_out * float(np.sqrt(h))

    def linearize(self, h_op: float):
        """Continuous (a, b) and discrete (Ad, Bd) model of the deviation dynamics at h_op.

        x = h - h_op, u = q_in - q_op with q_op = c sqrt(h_op). Returns (a, b, Ad, Bd).
        """
        a = -self.c_out / (2.0 * self.area * np.sqrt(h_op))
        b = 1.0 / self.area
        Ad, Bd = c2d_zoh([[a]], [[b]], self.Ts)
        return float(a), float(b), Ad, Bd
