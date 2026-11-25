"""Brushed DC motor with armature circuit and rotor mechanics.

    L di/dt = v - R i - Ke w
    J dw/dt = Kt i - B w - Tc tanh(w / eps) - Tl

v armature voltage (V), i armature current (A), w shaft speed (rad/s), Tl load torque (N m).
In SI units Kt (N m/A) and Ke (V s/rad) are numerically equal for an ideal motor.

Default parameters describe an illustrative 24 V gear-motor class machine. They are NOT measured
from a specific product. Replace them with datasheet or identified values for a real design.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace

import numpy as np

from control_lab.models.lti import rk4_step


@dataclass(frozen=True)
class DCMotorParams:
    R: float = 2.0  # armature resistance, ohm
    L: float = 3.0e-3  # armature inductance, H
    Kt: float = 0.08  # torque constant, N m / A
    Ke: float = 0.08  # back-EMF constant, V s / rad
    J: float = 4.0e-5  # rotor plus load inertia, kg m^2
    B: float = 2.0e-5  # viscous friction, N m s / rad
    Tc: float = 0.0  # Coulomb friction torque, N m
    v_max: float = 24.0  # supply limit, V

    def with_changes(self, **kw) -> DCMotorParams:
        return replace(self, **kw)

    def as_dict(self) -> dict:
        return asdict(self)

    @property
    def dc_gain(self) -> float:
        """Steady-state speed per volt at zero load torque, ignoring Coulomb friction (rad/s/V)."""
        return self.Kt / (self.R * self.B + self.Kt * self.Ke)

    @property
    def tau_mech(self) -> float:
        return self.J * self.R / (self.Kt * self.Ke + self.R * self.B)

    @property
    def tau_elec(self) -> float:
        return self.L / self.R

    def transfer_function_coeffs(self):
        """Speed / voltage transfer function (num, den) in descending powers of s.

        G(s) = Kt / (J L s^2 + (J R + B L) s + (R B + Kt Ke))
        """
        num = [self.Kt]
        den = [self.J * self.L, self.J * self.R + self.B * self.L, self.R * self.B + self.Kt * self.Ke]
        return num, den


class DCMotor:
    """Simulated motor. `step(v, tl)` holds v and tl constant over one controller sample."""

    def __init__(self, params: DCMotorParams | None = None, Ts: float = 1e-3, substeps: int = 10,
                 eps: float = 0.5):
        self.p = params or DCMotorParams()
        self.Ts, self.substeps, self.eps = float(Ts), int(substeps), float(eps)
        self.reset()

    def reset(self) -> None:
        self.x = np.zeros(2)  # [current, speed]

    @property
    def y(self) -> float:
        return float(self.x[1])

    @property
    def current(self) -> float:
        return float(self.x[0])

    def _f(self, x, u):
        v, tl = u
        p = self.p
        i, w = x
        di = (v - p.R * i - p.Ke * w) / p.L
        dw = (p.Kt * i - p.B * w - p.Tc * np.tanh(w / self.eps) - tl) / p.J
        return np.array([di, dw])

    def electromagnetic_torque(self) -> float:
        return self.p.Kt * self.current

    def step(self, v: float, tl: float = 0.0) -> float:
        v = float(np.clip(v, -self.p.v_max, self.p.v_max))
        dt = self.Ts / self.substeps
        x = self.x
        for _ in range(self.substeps):
            x = rk4_step(self._f, x, (v, tl), dt)
        self.x = x
        return self.y
