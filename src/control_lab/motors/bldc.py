"""Brushless DC motor with trapezoidal back-EMF and six-step (120 degree) commutation.

Model assumptions (stated so that results are not over-read):
  * star connection with isolated neutral, two phases conduct at a time
  * ideal commutation: the pair current is carried over instantly at a sector change, so the
    commutation torque dip caused by phase inductance is NOT represented
  * averaged PWM: the high-side phase is driven at duty D of the DC bus, the low side is on
  * no switch drops, dead time, iron loss or mutual inductance
With these assumptions the pair equations are
    2L dI/dt = D Vdc - 2R I - Ke w (f_h - f_l)        Te = Ke (f_h - f_l) I
where f_x is the unit trapezoidal back-EMF shape of phase x (see trapezoid_shape) and w is the
mechanical speed. Default parameters are illustrative, not from a datasheet.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace

import numpy as np

from control_lab.models.lti import rk4_step

# (high phase, low phase) per 60 degree sector; phases a=0, b=1, c=2.
SECTOR_PAIRS = [(0, 1), (0, 2), (1, 2), (1, 0), (2, 0), (2, 1)]
PHASE_SHIFT = (0.0, -2.0 * np.pi / 3.0, 2.0 * np.pi / 3.0)


def trapezoid_shape(theta_e, flat_deg: float = 120.0):
    """Unit trapezoidal back-EMF: clipped sine whose flat top is `flat_deg` wide.

    flat_deg = 120 gives the ideal 120 degree trapezoid (k = 2). A smaller value gives sloped
    tops, which creates torque ripple under six-step commutation.
    """
    if not (0.0 < flat_deg <= 180.0):
        raise ValueError("flat_deg must be in (0, 180]")
    half = np.deg2rad((180.0 - flat_deg) / 2.0)
    k = 1.0 / np.sin(half) if half > 0 else 1e6
    return np.clip(k * np.sin(theta_e), -1.0, 1.0)


def hall_sector(theta_e: float, offset_deg: float = 0.0) -> int:
    """Commutation sector 0..5 from the electrical angle, with a Hall mounting offset (deg)."""
    deg = (np.rad2deg(theta_e) - 30.0 - offset_deg) % 360.0
    return int(deg // 60.0) % 6


@dataclass(frozen=True)
class BLDCParams:
    R: float = 0.4  # phase resistance, ohm
    L: float = 0.8e-3  # phase inductance (self minus mutual), H
    Ke: float = 0.02  # phase peak back-EMF constant, V s / rad (mechanical)
    p: int = 4  # pole pairs
    J: float = 1.5e-5  # inertia, kg m^2
    B: float = 1.0e-6  # viscous friction, N m s / rad
    Vdc: float = 24.0  # DC bus, V
    flat_deg: float = 120.0  # width of the back-EMF flat top

    def with_changes(self, **kw) -> BLDCParams:
        return replace(self, **kw)

    def as_dict(self) -> dict:
        return asdict(self)

    @property
    def kt_pair(self) -> float:
        """Torque per ampere with an ideal trapezoid: 2 Ke (N m / A)."""
        return 2.0 * self.Ke


class BLDCMotor:
    """State [pair current I, mechanical speed w, mechanical angle theta]."""

    def __init__(self, params: BLDCParams | None = None, dt: float = 20e-6):
        self.p = params or BLDCParams()
        self.dt = float(dt)
        self.reset()

    def reset(self) -> None:
        self.x = np.zeros(3)
        self._te = 0.0

    @property
    def y(self) -> float:
        return float(self.x[1])

    @property
    def current(self) -> float:
        return float(self.x[0])

    @property
    def theta_e(self) -> float:
        return float(self.p.p * self.x[2])

    @property
    def torque(self) -> float:
        return self._te

    def _shapes(self, theta_e):
        return [trapezoid_shape(theta_e + s, self.p.flat_deg) for s in PHASE_SHIFT]

    def _f(self, x, u):
        duty, tl, hi, lo = u
        q = self.p
        i, w, th = x
        f = self._shapes(q.p * th)
        df = f[hi] - f[lo]
        di = (duty * q.Vdc - 2.0 * q.R * i - q.Ke * w * df) / (2.0 * q.L)
        te = q.Ke * df * i
        dw = (te - q.B * w - tl) / q.J
        return np.array([di, dw, w])

    def step(self, duty: float, tl: float = 0.0, hall_offset_deg: float = 0.0) -> float:
        """Advance one internal step `dt` with the pair chosen from the (offset) Hall sector."""
        duty = float(np.clip(duty, 0.0, 1.0))
        hi, lo = SECTOR_PAIRS[hall_sector(self.theta_e, hall_offset_deg)]
        self.x = rk4_step(self._f, self.x, (duty, tl, hi, lo), self.dt)
        f = self._shapes(self.theta_e)
        self._te = self.p.Ke * (f[hi] - f[lo]) * self.x[0]
        return self.y
