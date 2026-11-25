"""Permanent magnet synchronous motor in the rotor (dq) frame.

    Ld did/dt = vd - Rs id + we Lq iq
    Lq diq/dt = vq - Rs iq - we Ld id - we psi
    Te        = 1.5 p (psi iq + (Ld - Lq) id iq)
    J dwm/dt  = Te - B wm - Tl,   we = p wm,   dth/dt = wm

Voltages enter as a stationary-frame (alpha-beta) vector held constant over one control period,
and are rotated into dq at the instantaneous rotor angle inside the integrator, so the delay
between control and rotor motion is represented. Averaged inverter: no switching ripple, no
dead time, no iron loss, no saturation. Default parameters are illustrative, not from a datasheet.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace

import numpy as np

from control_lab.models.lti import rk4_step
from control_lab.transforms import inv_clarke, inv_park, park


@dataclass(frozen=True)
class PMSMParams:
    Rs: float = 0.35  # stator resistance, ohm
    Ld: float = 1.2e-3  # d-axis inductance, H
    Lq: float = 1.2e-3  # q-axis inductance, H (equal to Ld for a surface magnet machine)
    psi: float = 0.015  # permanent magnet flux linkage, Wb (peak, amplitude-invariant)
    p: int = 4  # pole pairs
    J: float = 5.0e-5  # inertia, kg m^2
    B: float = 1.0e-5  # viscous friction, N m s / rad
    Vdc: float = 48.0  # DC bus voltage, V

    def with_changes(self, **kw) -> PMSMParams:
        return replace(self, **kw)

    def as_dict(self) -> dict:
        return asdict(self)

    @property
    def kt(self) -> float:
        """Torque per q-axis ampere for id = 0: 1.5 p psi (N m / A)."""
        return 1.5 * self.p * self.psi

    @property
    def v_linear_max(self) -> float:
        """Largest phase-voltage amplitude in the linear SVPWM range."""
        return self.Vdc / np.sqrt(3.0)


class PMSM:
    def __init__(self, params: PMSMParams | None = None, Ts: float = 1e-4, substeps: int = 2):
        self.p = params or PMSMParams()
        self.Ts, self.substeps = float(Ts), int(substeps)
        self.reset()

    def reset(self) -> None:
        self.x = np.zeros(4)  # [id, iq, mechanical speed, mechanical angle]

    @property
    def y(self) -> float:
        return float(self.x[2])

    @property
    def theta_e(self) -> float:
        return float(self.p.p * self.x[3])

    @property
    def omega_e(self) -> float:
        return float(self.p.p * self.x[2])

    def torque(self, x=None) -> float:
        x = self.x if x is None else x
        q = self.p
        return 1.5 * q.p * (q.psi * x[1] + (q.Ld - q.Lq) * x[0] * x[1])

    def phase_currents(self):
        """Instantaneous phase currents (A) from the dq state, for use by a controller."""
        alpha, beta = inv_park(self.x[0], self.x[1], self.theta_e)
        return inv_clarke(alpha, beta)

    def _f(self, x, u):
        v_alpha, v_beta, tl = u
        q = self.p
        id_, iq, wm, th = x
        we = q.p * wm
        vd, vq = park(v_alpha, v_beta, q.p * th)
        did = (vd - q.Rs * id_ + we * q.Lq * iq) / q.Ld
        diq = (vq - q.Rs * iq - we * q.Ld * id_ - we * q.psi) / q.Lq
        te = 1.5 * q.p * (q.psi * iq + (q.Ld - q.Lq) * id_ * iq)
        dwm = (te - q.B * wm - tl) / q.J
        return np.array([did, diq, dwm, wm])

    def step(self, v_alpha: float, v_beta: float, tl: float = 0.0) -> float:
        dt = self.Ts / self.substeps
        x = self.x
        for _ in range(self.substeps):
            x = rk4_step(self._f, x, (v_alpha, v_beta, tl), dt)
        self.x = x
        return self.y
