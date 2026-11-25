"""Discrete positional PID with filtered derivative, setpoint weighting and anti-windup.

Control law (parallel form, gains in physical units):

    u = Kp (b r - y) + I + D + u_ff
    I_k = I_{k-1} + Ki Ts e_k                 e = r - y
    D_k = Tf/(Tf+Ts) D_{k-1} - Kd/(Tf+Ts) (y_k - y_{k-1})      (derivative on measurement)

b is the proportional setpoint weight (b = 1 is the textbook form). The derivative acts on the
measurement only, so setpoint steps do not create derivative kick. Tf = 0 removes the filter.

Anti-windup modes:
    "none"       plain integrator
    "clamp"      conditional integration: stop integrating while saturated and the error pushes
                 further into saturation
    "backcalc"   back-calculation with tracking time constant Tt (Astrom and Hagglund)
"""

from __future__ import annotations


class PID:
    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        Ts: float = 1e-3,
        u_min: float = float("-inf"),
        u_max: float = float("inf"),
        b: float = 1.0,
        tf: float = 0.0,
        antiwindup: str = "clamp",
        tt: float | None = None,
    ):
        if Ts <= 0:
            raise ValueError("Ts must be positive")
        if u_min > u_max:
            raise ValueError("u_min must not exceed u_max")
        if antiwindup not in ("none", "clamp", "backcalc"):
            raise ValueError("antiwindup must be 'none', 'clamp' or 'backcalc'")
        self.kp, self.ki, self.kd, self.Ts = float(kp), float(ki), float(kd), float(Ts)
        self.u_min, self.u_max, self.b, self.tf = float(u_min), float(u_max), float(b), float(tf)
        self.antiwindup = antiwindup
        if tt is None:
            tt = (self.kp / self.ki) if (self.ki > 0 and self.kp > 0) else 10 * self.Ts
        self.tt = float(tt)
        self.reset()

    @classmethod
    def from_times(cls, kp: float, ti: float, td: float = 0.0, **kw) -> PID:
        """Build from the textbook form Kp (1 + 1/(Ti s) + Td s)."""
        ki = kp / ti if ti and ti != float("inf") else 0.0
        return cls(kp=kp, ki=ki, kd=kp * td, **kw)

    def reset(self) -> None:
        self.i_term = 0.0
        self.d_term = 0.0
        self._y_prev = None
        self.u_unsat = 0.0
        self.u = 0.0

    def track(self, u_manual: float, r: float, y: float) -> None:
        """Bumpless transfer: set the integrator so that the next output equals `u_manual`."""
        p = self.kp * (self.b * r - y)
        self.i_term = u_manual - p - self.d_term
        self._y_prev = y
        self.u = u_manual

    def update(self, r: float, y: float, u_ff: float = 0.0) -> float:
        e = r - y
        p = self.kp * (self.b * r - y)
        if self._y_prev is None:
            self._y_prev = y
        dy = y - self._y_prev
        self._y_prev = y
        if self.kd != 0.0:
            self.d_term = self.tf / (self.tf + self.Ts) * self.d_term - self.kd / (
                self.tf + self.Ts
            ) * dy
        else:
            self.d_term = 0.0

        i_new = self.i_term + self.ki * self.Ts * e
        u_try = p + i_new + self.d_term + u_ff
        u_sat = min(max(u_try, self.u_min), self.u_max)

        if self.antiwindup == "none":
            self.i_term = i_new
        elif self.antiwindup == "clamp":
            pushing = (u_try > self.u_max and e > 0) or (u_try < self.u_min and e < 0)
            if not pushing:
                self.i_term = i_new
        else:  # back-calculation
            self.i_term = i_new + self.Ts / self.tt * (u_sat - u_try)

        self.u_unsat = u_try
        self.u = u_sat
        return u_sat
