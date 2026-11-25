"""Field-oriented control building blocks for a PMSM: dq current controller and gain design.

Current controller (per axis): PI with pole-zero cancellation of the stator R-L plant,
    Kp = L wc,  Ki = R wc
gives a first-order closed loop with bandwidth wc (rad/s) for an ideal, delay-free current loop.
Cross-coupling terms are feed-forward compensated. The voltage vector is limited to a circle of
radius `v_max` (priority is kept in direction, not on d or q) and the integrators use
back-calculation so they do not wind up while the voltage is limited.
"""

from __future__ import annotations

import math


def current_loop_gains(Ld: float, Lq: float, Rs: float, bandwidth_hz: float):
    """Return (kp_d, ki_d, kp_q, ki_q) for the requested current-loop bandwidth."""
    wc = 2.0 * math.pi * bandwidth_hz
    return Ld * wc, Rs * wc, Lq * wc, Rs * wc


def speed_loop_gains(J: float, kt: float, bandwidth_hz: float):
    """PI gains for a speed loop around an ideal torque source: plant kt/(J s).

    Closed-loop polynomial s^2 + (kt kp / J) s + kt ki / J with a double pole at wn:
    kp = 2 J wn / kt, ki = J wn^2 / kt. Friction and current-loop lag are neglected, so the
    result is a starting point that must be checked in simulation.
    """
    wn = 2.0 * math.pi * bandwidth_hz
    return 2.0 * J * wn / kt, J * wn * wn / kt


class DQCurrentController:
    def __init__(self, Ld, Lq, Rs, psi, Ts, bandwidth_hz, v_max, decouple=True):
        self.Ld, self.Lq, self.Rs, self.psi, self.Ts = Ld, Lq, Rs, psi, Ts
        self.kp_d, self.ki_d, self.kp_q, self.ki_q = current_loop_gains(Ld, Lq, Rs, bandwidth_hz)
        self.v_max, self.decouple = float(v_max), decouple
        self.reset()

    def reset(self):
        self.int_d = 0.0
        self.int_q = 0.0
        self.saturated = False

    def update(self, id_ref, iq_ref, id_, iq, omega_e):
        """Return (vd, vq) in volts."""
        ed, eq = id_ref - id_, iq_ref - iq
        ff_d = -omega_e * self.Lq * iq if self.decouple else 0.0
        ff_q = omega_e * (self.Ld * id_ + self.psi) if self.decouple else 0.0
        vd_u = self.kp_d * ed + self.int_d + ff_d
        vq_u = self.kp_q * eq + self.int_q + ff_q
        mag = math.hypot(vd_u, vq_u)
        scale = 1.0 if mag <= self.v_max else self.v_max / mag
        vd, vq = vd_u * scale, vq_u * scale
        self.saturated = scale < 1.0
        # integrate error, plus back-calculation on the amount removed by the limiter
        tt = max(self.kp_d / max(self.ki_d, 1e-12), self.Ts)
        self.int_d += self.ki_d * self.Ts * ed + self.Ts / tt * (vd - vd_u)
        self.int_q += self.ki_q * self.Ts * eq + self.Ts / tt * (vq - vq_u)
        return vd, vq
