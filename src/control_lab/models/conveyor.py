"""Belt conveyor, lumped single-mass model with a drive-force lag.

    tau_t dF/dt = F_max u - F                                   (drive force, first-order lag)
    m dv/dt     = F - c v - mu m g tanh(v / eps) - d            (belt speed)
    m           = m_belt + m_payload

v belt speed (m/s, non-negative direction of travel), F drive force at the belt (N), u normalised
drive command in [0, 1], d external resisting force (N, the disturbance input of `step`).
`m_belt` is the equivalent mass of belt, rollers and the motor inertia reflected to the belt.
`c` lumps viscous losses of belt, gearbox and motor.

Payload changes: `drop_item(dm)` with dm > 0 adds a mass that starts at rest and is captured
instantly (perfectly inelastic), so momentum of the equivalent mass is conserved:
v+ = m v / (m + dm). A negative dm removes mass that travels with the belt, so v is unchanged.
Real items slide for a short time before they match belt speed; that is not modelled.

All default parameters are illustrative for a bench-scale belt, not measured from a product.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace

import numpy as np

from control_lab.models.lti import rk4_step

G = 9.80665


@dataclass(frozen=True)
class ConveyorParams:
    m_belt: float = 12.0  # equivalent mass of belt, rollers, reflected motor inertia, kg
    c: float = 40.0  # viscous loss coefficient, N s / m
    mu: float = 0.03  # rolling resistance coefficient (dimensionless)
    F_max: float = 40.0  # drive force at u = 1, N
    tau_t: float = 0.05  # drive force time constant, s
    eps: float = 0.01  # speed scale of the smooth friction sign function, m/s

    def with_changes(self, **kw) -> ConveyorParams:
        return replace(self, **kw)

    def as_dict(self) -> dict:
        return asdict(self)


class BeltConveyor:
    """Simulated conveyor. Follows the plant protocol of `control_lab.simulation`.

    payload_events: optional list of (time_s, delta_mass_kg) applied at the first sample whose start
    time is at or after the event time.
    v_init: initial belt speed. The drive force starts at the value that holds this speed, so the
    belt begins in steady state (with the initial payload), not from rest.
    """

    def __init__(self, params: ConveyorParams | None = None, Ts: float = 5e-3, substeps: int = 5,
                 m_payload: float = 0.0, payload_events=None, v_init: float = 0.0):
        if Ts <= 0 or substeps < 1:
            raise ValueError("Ts must be positive and substeps >= 1")
        self.p = params or ConveyorParams()
        self.Ts, self.substeps, self.m_payload0 = float(Ts), int(substeps), float(m_payload)
        self.events0 = sorted(payload_events or [])
        self.v_init = float(v_init)
        self.reset()

    def reset(self) -> None:
        self.m_payload = self.m_payload0
        v = self.v_init
        f_hold = self.p.c * v + self.p.mu * self.mass * G * np.tanh(v / self.p.eps)
        self.x = np.array([v, f_hold])  # [speed, drive force]
        self.t = 0.0
        self._events = list(self.events0)

    @property
    def y(self) -> float:
        return float(self.x[0])

    @property
    def force(self) -> float:
        return float(self.x[1])

    @property
    def mass(self) -> float:
        return self.p.m_belt + self.m_payload

    def drop_item(self, dm: float) -> None:
        if dm > 0:
            m = self.mass
            self.x[0] = self.x[0] * m / (m + dm)
            self.m_payload += dm
        else:
            self.m_payload = max(0.0, self.m_payload + dm)

    def _f(self, x, u):
        v, F = x
        cmd, d, m = u
        p = self.p
        dF = (p.F_max * cmd - F) / p.tau_t
        dv = (F - p.c * v - p.mu * m * G * np.tanh(v / p.eps) - d) / m
        return np.array([dv, dF])

    def step(self, u: float, d: float = 0.0) -> float:
        while self._events and self._events[0][0] <= self.t + 1e-12:
            self.drop_item(self._events.pop(0)[1])
        cmd = float(np.clip(u, 0.0, 1.0))
        dt = self.Ts / self.substeps
        x = self.x.copy()
        for _ in range(self.substeps):
            x = rk4_step(self._f, x, (cmd, d, self.mass), dt)
            if x[0] < 0.0:  # the belt does not run backwards in this model
                x[0] = 0.0
        self.x = x
        self.t += self.Ts
        return self.y

    # analytic helpers used as independent references

    def steady_speed(self, u: float, m_payload: float | None = None) -> float:
        """Steady speed for constant command u (valid when v >> eps), clipped at zero."""
        mp = self.m_payload if m_payload is None else m_payload
        m = self.p.m_belt + mp
        return max(0.0, (self.p.F_max * float(np.clip(u, 0, 1)) - self.p.mu * m * G) / self.p.c)

    def command_for_speed(self, v: float, m_payload: float | None = None) -> float:
        """Steady command that holds speed v (may exceed 1 when v is not reachable)."""
        mp = self.m_payload if m_payload is None else m_payload
        m = self.p.m_belt + mp
        return (self.p.c * v + self.p.mu * m * G) / self.p.F_max

    def max_speed(self, m_payload: float | None = None) -> float:
        return self.steady_speed(1.0, m_payload)

    def linear_tf(self, m_payload: float | None = None):
        """Speed/command transfer function (num, den) of the friction-free linearisation.

        G(s) = (F_max/c) / ((tau_t s + 1)(tau_m s + 1)),  tau_m = m / c.
        Exact for mu = 0; with mu > 0 the incremental gain at v >> eps is the same.
        """
        mp = self.m_payload if m_payload is None else m_payload
        tau_m = (self.p.m_belt + mp) / self.p.c
        K = self.p.F_max / self.p.c
        tt = self.p.tau_t
        return [K], [tt * tau_m, tt + tau_m, 1.0]
